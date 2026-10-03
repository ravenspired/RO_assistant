# main.py - ESP32-C3 RO water tank monitor (HX711 + 20 kg load cell), MicroPython
# Live dashboard: ml remaining, % full, ml dispensed per session, refill rate (ml/s),
# and a live time-to-full countdown. The tank sits on the load cell under the sink.
#
# Wiring (ESP32-C3 Super Mini):
#   HX711 VCC -> 3V3   (keep it at 3.3 V so DOUT is safe for the C3)
#   HX711 GND -> GND
#   HX711 DT  -> GPIO4
#   HX711 SCK -> GPIO5
#   Load cell: red=E+, black=E-, white=A-, green=A+  (reversed polarity is fine,
#   the calibration handles a negative sign)
#
# How it works
#   Calibration (done once from the web page):
#     1. capture FULL weight  2. close feed valve, drain tank, capture EMPTY weight
#     3. enter grams of water collected (gives counts/gram AND tank capacity)
#     4. open valve: the refill rate (ml/s) vs fullness (20 bins of 5%) is logged
#   Normal use:
#     The feed valve stays open, so the tank refills while you draw water.
#     Dispensed = (level drop) + (expected refill during the draw), where the
#     expected refill comes from the learned rate-vs-fullness profile.
#     A session starts when the tank drains faster than it refills and ends
#     after 10 s with no draw. The profile keeps learning (EMA) whenever the
#     tank is idle, so it follows changes in line pressure / water temperature.
#   Everything is stored in tank.json on the board (calibration + refill profile).

import gc
import json
import time
import network
import machine
from machine import Pin

try:
    import asyncio
except ImportError:
    import uasyncio as asyncio

# ------------------------------ CONFIG ------------------------------
SSID = "YOUR_WIFI_NAME"
PASSWORD = "YOUR_WIFI_PASSWORD"
HOSTNAME = "rotank"
TX_POWER = 8.5            # dBm, helps flaky Super Mini antennas. None = default
AP_SSID = "ROTank"        # fallback hotspot if WiFi fails at boot (IP 192.168.4.1)
AP_PASSWORD = "rotank1234"

HX_DOUT = 4
HX_SCK = 5

TICK_MS = 500             # engine update period (averages ~5 HX711 samples)
HIST_N = 130              # history length (130 x 0.5 s = 65 s)
DW = 2                    # seconds, window used to detect a draw
DRAW_ML_S = 1.0           # draw detected if (expected refill - measured rate) > this
SESSION_END_MS = 10000    # session ends after this long with no draw
MIN_SESSION_ML = 10       # ignore "sessions" smaller than this (bumps, vibration)
NB = 20                   # refill-profile bins (5% each)
FULL_FRAC = 0.98          # >= this fraction counts as full
FULL_RATE = 0.05          # ml/s; profile rate below this (when >60% full) = "full"
LEARN_MS = 10000          # refill-rate learning window
SAVE_MS = 600000          # save learned profile at most this often
CAPTURE_N = 30            # samples per calibration capture (~3 s)
DRIFT_G = 2.0             # reject a capture if the reading moved more than this
DEFAULT_CPG = 215.0       # rough counts/gram, only used for the drift check
MIN_SPAN = 20000          # min raw-count difference between full and empty
CAL_FILE = "tank.json"
# --------------------------------------------------------------------


class HX711:
    def __init__(self, dout, sck, gain=128):
        self.dout = Pin(dout, Pin.IN)
        self.sck = Pin(sck, Pin.OUT, value=0)
        self.extra = {128: 1, 32: 2, 64: 3}[gain]

    def ready(self):
        return self.dout.value() == 0

    def read(self):
        """Read one 24-bit signed sample. Call only when ready()."""
        dout = self.dout
        sck = self.sck
        v = 0
        irq = machine.disable_irq()
        for _ in range(24):
            sck.value(1)
            sck.value(0)
            v = (v << 1) | dout.value()
        for _ in range(self.extra):
            sck.value(1)
            sck.value(0)
        machine.enable_irq(irq)
        if v & 0x800000:
            v -= 0x1000000
        return v


def mean8(vals):
    """Trimmed mean of ints, returned as an int scaled x8 (keeps precision
    without relying on MicroPython's low-precision floats)."""
    v = sorted(vals)
    n = len(v)
    k = n // 5
    return sum(v[k:n - k]) * 8 // (n - 2 * k)


class Tank:
    def __init__(self):
        self.hx = HX711(HX_DOUT, HX_SCK)
        now = time.ticks_ms()
        self.seq = 0
        self.last_raw = 0
        self.last_ms = now
        self.acc = []
        self.t_prev = now
        self.raw = 0
        # calibration
        self.full8 = 0
        self.empty8 = 0
        self.cap = 0.0          # tank capacity in ml (grams collected)
        self.k = 0.0            # ml per raw8 count
        self.prof = [None] * NB  # refill rate (ml/s) per 5% of fullness
        self.dirty = False
        self.step = None        # None, full, drain, grams, refill
        self.w = {}
        self.busy = False
        # live values
        self.L = 0.0            # ml above "empty"
        self.hist = []          # [(ticks_ms, ml)]
        self.rate = 0.0         # measured refill rate, ml/s
        self.full = False
        self.eta = None
        self.flow = 0.0
        self.t_learn = now
        self.t_save = now
        self.last_draw = time.ticks_add(now, -60000)
        self.sessions = []      # newest first: [ml, seconds]
        self.reset_session()
        self.load()

    # ---------------- storage ----------------
    def set_cal(self, full8, empty8, cap):
        k = float(cap) / (full8 - empty8)
        self.full8 = full8
        self.empty8 = empty8
        self.cap = float(cap)
        self.k = k

    def load(self):
        try:
            with open(CAL_FILE) as f:
                d = json.load(f)
            self.set_cal(d["full"], d["empty"], d["cap"])
            p = d.get("prof")
            if p and len(p) == NB:
                self.prof = p
        except Exception:
            pass

    def save(self):
        self.t_save = time.ticks_ms()
        try:
            with open(CAL_FILE, "w") as f:
                json.dump({"full": self.full8, "empty": self.empty8,
                           "cap": self.cap, "prof": self.prof}, f)
            self.dirty = False
        except OSError:
            pass

    def clear_all(self):
        self.cap = 0.0
        self.k = 0.0
        self.prof = [None] * NB
        self.step = None
        self.w = {}
        self.hist = []
        self.L = 0.0
        self.full = False
        self.eta = None
        self.reset_session()
        try:
            import os
            os.remove(CAL_FILE)
        except OSError:
            pass

    def reset_session(self):
        self.sess_active = False
        self.sess_ml = 0.0
        self.sess_L0 = 0.0
        self.sess_refill = 0.0
        self.sess_t0 = 0
        self.sess_snap = 0.0
        self.sess_dur = 0.0
        self.flow = 0.0

    # ---------------- history helpers ----------------
    def back(self, secs):
        """(elapsed_ms, level_ml) of the newest history entry at least secs old."""
        h = self.hist
        if len(h) < 2:
            return None
        t1 = h[-1][0]
        want = secs * 1000
        for i in range(len(h) - 2, -1, -1):
            dt = time.ticks_diff(t1, h[i][0])
            if dt >= want:
                return dt, h[i][1]
        return None

    def slope(self, secs):
        """Level change in ml/s over the last `secs` seconds (None if not enough data)."""
        got = self.back(secs)
        if got is None:
            return None
        return (self.L - got[1]) * 1000 / got[0]

    # ---------------- refill profile ----------------
    def rate_at(self, f):
        """Expected refill rate (ml/s) at fullness fraction f, interpolated."""
        p = self.prof
        x = f * NB - 0.5
        i0 = int(x) if x >= 0 else -1
        lo = None
        hi = None
        for i in range(min(i0, NB - 1), -1, -1):
            if p[i] is not None:
                lo = i
                break
        for i in range(max(i0 + 1, 0), NB):
            if p[i] is not None:
                hi = i
                break
        if lo is None and hi is None:
            return None
        if lo is None:
            return p[hi]
        if hi is None or hi == lo:
            return p[lo]
        w = (x - lo) / (hi - lo)
        return p[lo] + (p[hi] - p[lo]) * w

    def learn(self, now):
        """Update the rate-vs-fullness profile from the last 10 s of undisturbed data."""
        if time.ticks_diff(now, self.t_learn) < LEARN_MS:
            return
        got = self.back(10)
        if got is None:
            return
        self.t_learn = now
        dt, L0 = got
        s = (self.L - L0) * 1000 / dt
        if s < -0.15:
            return                      # being drawn from, not refilling
        mid = (self.L + L0) / 2 / self.cap
        if s < 0.02 and mid < 0.93:
            return                      # valve closed / no supply: don't learn zeros
        b = int(mid * NB)
        if b < 0:
            b = 0
        if b >= NB:
            b = NB - 1
        v = s if s > 0 else 0.0
        old = self.prof[b]
        self.prof[b] = v if old is None else old * 0.7 + v * 0.3
        self.dirty = True

    def eta_s(self):
        """Seconds until the refill rate has decayed to ~0 (tank effectively full)."""
        cap = self.cap
        x = self.L / cap
        if x < 0:
            x = 0.0
        t = 0.0
        for _ in range(110):
            if x >= FULL_FRAC - 1e-4:
                break
            w = 0.01
            if x + w > FULL_FRAC:
                w = FULL_FRAC - x
            r = self.rate_at(x + w / 2)
            if r is None:
                return None
            if r < FULL_RATE:
                if x > 0.6:
                    break               # shut-off region: treat as full
                r = FULL_RATE
            t += cap * w / r
            x += w
        return t

    def is_full(self):
        if self.L >= FULL_FRAC * self.cap:
            return True
        if self.L > 0.85 * self.cap:
            s = self.slope(60)
            if s is not None and -0.02 < s < 0.02:
                return True
        return False

    # ---------------- sampling ----------------
    def ingest(self, raw):
        self.seq += 1
        self.last_raw = raw
        self.last_ms = time.ticks_ms()
        self.acc.append(raw)
        if len(self.acc) > 40:
            self.acc.pop(0)

    async def run(self):
        hx = self.hx
        t_tick = time.ticks_ms()
        while True:
            if hx.ready():
                try:
                    self.ingest(hx.read())
                except Exception:
                    pass
            now = time.ticks_ms()
            if time.ticks_diff(now, t_tick) >= TICK_MS and len(self.acc) >= 2:
                t_tick = now
                r8 = mean8(self.acc)
                self.acc = []
                try:
                    self.tick(now, r8)
                except Exception as e:
                    print("tick error:", e)
            await asyncio.sleep_ms(5)

    # ---------------- the engine ----------------
    def tick(self, now, r8):
        dt = time.ticks_diff(now, self.t_prev) / 1000.0
        self.t_prev = now
        self.raw = r8 // 8
        if self.cap <= 0:
            return
        L = (r8 - self.empty8) * self.k
        self.L = L
        h = self.hist
        h.append((now, L))
        if len(h) > HIST_N:
            h.pop(0)

        s = self.slope(8)
        self.rate = s if s is not None else 0.0
        self.full = self.is_full()

        if self.step:
            self.reset_session()
            if self.step == "refill":
                self.learn(now)
                if self.full:           # refill logging complete
                    self.step = None
                    self.save()
        else:
            self.track(now, dt, L)
            if not self.sess_active and time.ticks_diff(now, self.last_draw) > 15000:
                self.learn(now)

        if not self.sess_active and not self.full:
            self.eta = self.eta_s()
        else:
            self.eta = None

        if self.dirty and time.ticks_diff(now, self.t_save) > SAVE_MS:
            self.save()

    def track(self, now, dt, L):
        """Detect draws and integrate dispensed volume, accounting for refill."""
        r = self.rate_at(L / self.cap)
        if r is None or r < 0:
            r = 0.0
        s = self.slope(DW)
        drawing = s is not None and (r - s) > DRAW_ML_S
        if drawing:
            self.flow = r - s
            self.last_draw = now
        else:
            self.flow = 0.0

        if drawing and not self.sess_active:
            dt0, L0 = self.back(DW)
            self.sess_active = True
            self.sess_L0 = L0
            self.sess_refill = r * dt0 / 1000.0
            self.sess_t0 = time.ticks_add(now, -dt0)
            self.sess_snap = 0.0
            self.sess_dur = 0.0
        elif self.sess_active:
            self.sess_refill += r * dt

        if self.sess_active:
            ml = self.sess_L0 - L + self.sess_refill
            if ml < 0:
                ml = 0.0
            self.sess_ml = ml
            if drawing:
                self.sess_snap = ml
                self.sess_dur = time.ticks_diff(now, self.sess_t0) / 1000.0
            elif time.ticks_diff(now, self.last_draw) > SESSION_END_MS:
                if self.sess_snap >= MIN_SESSION_ML:
                    self.sessions.insert(0, [round(self.sess_snap), round(self.sess_dur)])
                    del self.sessions[5:]
                self.sess_active = False
                self.sess_ml = 0.0

    # ---------------- calibration wizard ----------------
    async def capture(self):
        vals = []
        last = self.seq
        t0 = time.ticks_ms()
        while len(vals) < CAPTURE_N + 3:
            if self.seq != last:
                last = self.seq
                vals.append(self.last_raw)
            if time.ticks_diff(time.ticks_ms(), t0) > 15000:
                return None, "HX711 not responding"
            await asyncio.sleep_ms(15)
        vals = vals[3:]
        h = len(vals) // 3
        if abs(mean8(vals[:h]) - mean8(vals[-h:])) > DRIFT_G * DEFAULT_CPG * 8:
            return None, "Reading was drifting (still filling or dripping?) - wait and retry"
        return mean8(vals), None

    async def wiz_capture(self, which):
        need = {"full": "full", "empty": "drain"}.get(which)
        if need is None or self.step != need:
            return "Wrong step"
        if self.busy:
            return "Busy"
        self.busy = True
        try:
            raw8, err = await self.capture()
        finally:
            self.busy = False
        if err:
            return err
        if self.step != need:
            return "Cancelled"
        if which == "full":
            self.w["full"] = raw8
            self.step = "drain"
        else:
            if abs(raw8 - self.w["full"]) < MIN_SPAN * 8:
                return "Empty reading is nearly the same as full - is the tank drained?"
            self.w["empty"] = raw8
            self.step = "grams"
        return None

    def wiz_grams(self, g):
        if self.step != "grams":
            return "Wrong step"
        if g < 200 or g > 15000:
            return "Enter the collected water in grams (200 - 15000)"
        self.set_cal(self.w["full"], self.w["empty"], g)
        self.prof = [None] * NB
        self.hist = []
        self.w = {}
        self.step = "refill"
        self.save()
        return None

    async def api_cal(self, act, q):
        if act == "start":
            self.step = "full"
            self.w = {}
            self.reset_session()
            return None
        if act == "cancel":
            self.step = None
            self.w = {}
            return None
        if act == "capture":
            return await self.wiz_capture(q.get("which", ""))
        if act == "grams":
            try:
                g = float(q.get("g", "0"))
            except ValueError:
                g = 0.0
            return self.wiz_grams(g)
        if act == "finish":
            if self.step == "refill":
                self.step = None
                self.save()
                return None
            return "Not in the refill step"
        if act == "clear":
            self.clear_all()
            return None
        return "Unknown action"

    # ---------------- state for the web page ----------------
    def mode(self):
        if self.step:
            return "calibrating"
        if self.cap <= 0:
            return "uncal"
        if self.sess_active:
            return "dispensing"
        if self.full:
            return "full"
        if self.rate > 0.03:
            return "refilling"
        return "idle"

    def state(self):
        err = None
        if time.ticks_diff(time.ticks_ms(), self.last_ms) > 2000:
            err = "HX711 not responding - check wiring"
        elif self.last_raw in (8388607, -8388608):
            err = "HX711 saturated - check load cell wiring / overload"
        d = {
            "mode": self.mode(),
            "err": err,
            "raw": self.raw,
            "cal": self.step,
            "cap": round(self.cap),
            "prof": [None if p is None else round(p, 3) for p in self.prof],
            "hist": self.sessions,
        }
        if self.cap > 0:
            L = self.L if self.L > 0 else 0.0
            pct = 100.0 * L / self.cap
            d["ml"] = round(L)
            d["pct"] = round(pct if pct < 100 else 100.0, 1)
            d["rate"] = round(self.rate if self.rate > 0 else 0.0, 3)
            d["eta"] = None if self.eta is None else round(self.eta)
            d["flow"] = round(self.flow, 1)
            d["sess"] = round(self.sess_ml)
            d["sess_s"] = round(self.sess_dur)
        return d


INDEX = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RO Tank</title>
<style>
:root{color-scheme:dark;--ac:#0a84ff}
body{margin:0;padding:16px;font-family:system-ui,sans-serif;background:#111;color:#eee;display:flex;flex-direction:column;align-items:center}
.card{background:#1c1c1e;border-radius:16px;padding:18px;width:100%;max-width:460px;margin-bottom:14px;box-sizing:border-box;text-align:center}
.pill{display:inline-block;padding:4px 14px;border-radius:99px;background:#2c2c2e;font-size:13px;font-weight:600;letter-spacing:.6px;text-transform:uppercase}
.pill.dispensing{background:#0a84ff}
.pill.refilling{background:#30d158;color:#000}
.pill.full{background:#3ddc84;color:#000}
.pill.calibrating{background:#ff9f0a;color:#000}
.pill.uncal{background:#5a3b00;color:#ffd48a}
#lab{color:#999;font-size:14px;margin-top:6px;min-height:18px}
#big{font-size:clamp(36px,13vw,58px);font-weight:600;font-variant-numeric:tabular-nums;margin:2px 0}
#sub{color:#999;font-size:14px;min-height:20px}
#banner{display:none;background:#5a1010;color:#ffb0b0;border-radius:10px;padding:10px;font-size:14px;margin-bottom:10px}
.row{display:flex;gap:8px;margin-top:12px}
button{flex:1;padding:14px;border:0;border-radius:12px;background:#2c2c2e;color:#eee;font-size:16px}
button.p{background:#0a84ff;color:#fff}
.in{display:none;gap:8px;margin-top:10px}
input{flex:1;background:#2c2c2e;border:0;border-radius:10px;color:#eee;padding:12px;font-size:16px;min-width:0}
#wz_p{color:#bbb;font-size:14px;line-height:1.4;margin:8px 0 0}
#msg{font-size:14px;min-height:20px;color:#ffd48a;margin-top:8px}
#hist div{display:flex;justify-content:space-between;padding:7px 0;border-top:1px solid #2c2c2e}
#hist span{color:#999}
#prof{display:flex;align-items:flex-end;gap:2px;height:80px;margin-top:10px}
.bar{flex:1;background:var(--ac);border-radius:2px 2px 0 0}
small{color:#777;display:block;margin-top:6px}
</style></head><body>
<div class="card">
<div id="banner"></div>
<span class="pill" id="pill">...</span>
<div><svg viewBox="0 0 200 200" width="230" height="230">
<circle cx="100" cy="100" r="84" fill="none" stroke="#2c2c2e" stroke-width="16"/>
<circle id="ring" cx="100" cy="100" r="84" fill="none" stroke="#0a84ff" stroke-width="16" stroke-linecap="round" stroke-dasharray="527.8" stroke-dashoffset="527.8" transform="rotate(-90 100 100)" style="transition:stroke-dashoffset .5s"/>
<text id="pct" x="100" y="108" text-anchor="middle" font-size="44" font-weight="600" fill="#eee">--</text>
<text id="rem" x="100" y="132" text-anchor="middle" font-size="15" fill="#aaa"></text>
</svg></div>
<div id="lab"></div><div id="big"></div><div id="sub"></div>
</div>

<div class="card"><b>Recent sessions</b><div id="hist" style="margin-top:8px;text-align:left"></div></div>

<div class="card"><b id="wz_t">Calibration</b>
<p id="wz_p"></p>
<div class="in" id="wz_in"><input id="gin" inputmode="decimal" placeholder="grams (ml) collected"></div>
<div class="row"><button class="p" id="wz_b"></button><button id="wz_c"></button></div>
<div id="msg"></div></div>

<div class="card"><b>Refill rate vs fullness</b><div id="prof"></div>
<small>Each bar = 5% of the tank, left = empty. Learned automatically (ml/s).</small></div>

<script>
const $=id=>document.getElementById(id);
const C=527.8;
const f0=n=>Math.round(n).toLocaleString();
const pad=n=>String(n).padStart(2,'0');
function dur(s){s=Math.max(0,Math.round(s));const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return (h?h+':'+pad(m):m)+':'+pad(s%60)}
const NAMES={uncal:'Not calibrated',calibrating:'Calibrating',dispensing:'Dispensing',refilling:'Refilling',full:'Tank full',idle:'Idle'};
let mode='',tgt=null,lastStep='x',lastProf='',busy=false;

async function call(p){
  if(busy)return;busy=true;
  $('msg').textContent=p.indexOf('capture')>0?'Capturing... keep everything still (~5 s)':'';
  try{const r=await fetch(p,{method:'POST'});const d=await r.json();$('msg').textContent=d.ok?'':('Error: '+d.err)}
  catch(e){$('msg').textContent='Request failed'}
  busy=false;
}
function grams(){
  const g=parseFloat($('gin').value);
  if(!(g>0)){$('msg').textContent='Enter the collected water first';return}
  call('/api/cal/grams?g='+g);
}
const WZ={
 full:['Step 1/4 - Capture FULL weight','The tank must be completely full and filling must have stopped. Keep everything still.','Capture full weight',()=>call('/api/cal/capture?which=full')],
 drain:['Step 2/4 - Drain the tank','First CLOSE the RO supply valve so no more water enters. Then drain the entire tank into a container until it is completely empty and nothing is dripping.','Capture empty weight',()=>call('/api/cal/capture?which=empty')],
 grams:['Step 3/4 - Water collected','Enter exactly how much water you drained (1 g = 1 ml). Weigh it or use a measuring jug.','Save',grams],
 refill:['Step 4/4 - Log the refill','OPEN the supply valve now and do not draw water. The refill rate vs fullness is recorded below and logging finishes by itself when the tank is full (can take hours).','Finish now',()=>call('/api/cal/finish')]
};
function setWiz(s){
  const w=WZ[s];
  $('wz_in').style.display=s==='grams'?'flex':'none';
  if(w){
    $('wz_t').textContent=w[0];$('wz_p').textContent=w[1];
    $('wz_b').textContent=w[2];$('wz_b').onclick=w[3];
    $('wz_c').textContent='Cancel';$('wz_c').onclick=()=>call('/api/cal/cancel');
  }else{
    $('wz_t').textContent='Calibration';
    $('wz_b').textContent='Start calibration';$('wz_b').onclick=()=>call('/api/cal/start');
    $('wz_c').textContent='Clear';
    $('wz_c').onclick=()=>{if(confirm('Erase calibration and refill profile?'))call('/api/cal/clear')};
  }
}
function drawProf(p){
  const sig=JSON.stringify(p);if(sig===lastProf)return;lastProf=sig;
  const mx=Math.max(0.1,...p.map(v=>v||0));
  $('prof').innerHTML=p.map((v,i)=>'<div class="bar" title="'+(i*5)+'-'+(i*5+5)+'%: '+(v==null?'?':v.toFixed(2)+' ml/s')+'" style="height:'+(v==null?2:Math.max(2,v/mx*100))+'%"></div>').join('');
}
function render(d){
  const b=$('banner');
  if(d.err){b.style.display='block';b.textContent=d.err}else b.style.display='none';
  mode=d.mode;
  $('pill').textContent=NAMES[mode];$('pill').className='pill '+mode;
  const has=d.pct!==undefined;
  $('ring').style.strokeDashoffset=has?C*(1-d.pct/100):C;
  $('pct').textContent=has?d.pct.toFixed(0)+'%':'--';
  $('rem').textContent=has?f0(d.ml)+' ml left':'';
  let lab='',big='',sub='';
  if(mode==='dispensing'){
    lab='Dispensed this session';big=f0(d.sess)+' ml';
    sub=d.flow.toFixed(1)+' ml/s  |  '+d.sess_s+' s';
  }else if(mode==='refilling'){
    lab='Full in';
    if(d.eta!=null){const t=Date.now()+d.eta*1000;if(tgt===null||Math.abs(t-tgt)>8000)tgt=t;else tgt+=(t-tgt)*0.1}else tgt=null;
    big=tgt?dur((tgt-Date.now())/1000):'--';
    sub='Filtering '+d.rate.toFixed(2)+' ml/s  ('+(d.rate*60).toFixed(0)+' ml/min)';
  }else if(mode==='full'){lab='Ready';big='Full'}
  else if(mode==='idle'){lab='Not refilling';big='--';sub='Supply valve closed or no pressure?'}
  else if(mode==='calibrating'&&d.rate>0.03){lab='Refill logging';big=d.rate.toFixed(2)+' ml/s'}
  if(mode!=='refilling')tgt=null;
  $('lab').textContent=lab;$('big').textContent=big;$('sub').textContent=sub;
  $('hist').innerHTML=(d.hist||[]).map(h=>'<div><b>'+f0(h[0])+' ml</b><span>'+h[1]+' s</span></div>').join('')||'<div><span>No sessions yet</span></div>';
  if(d.cal!==lastStep){lastStep=d.cal;setWiz(d.cal)}
  if(!d.cal)$('wz_p').textContent=has?'Calibrated: tank capacity '+f0(d.cap)+' ml. Recalibrate if you move or change the tank.':'Calibrate once to get started: fill, drain and refill the tank.';
  drawProf(d.prof);
}
setInterval(()=>{if(mode==='refilling'&&tgt)$('big').textContent=dur((tgt-Date.now())/1000)},250);
async function poll(){
  try{const r=await fetch('/api/state',{cache:'no-store'});render(await r.json())}
  catch(e){$('banner').style.display='block';$('banner').textContent='connection lost...'}
  setTimeout(poll,500);
}
poll();
</script></body></html>"""


def parse_query(q):
    d = {}
    for kv in q.split("&"):
        if "=" in kv:
            k, v = kv.split("=", 1)
            d[k] = v
    return d


async def send(w, body, ctype="application/json", status="200 OK"):
    if isinstance(body, str):
        body = body.encode()
    hdr = ("HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
           "Cache-Control: no-store\r\nConnection: close\r\n\r\n") % (status, ctype, len(body))
    w.write(hdr.encode())
    w.write(body)
    await w.drain()


tank = Tank()


async def handle(r, w):
    try:
        req = await asyncio.wait_for(r.readline(), 5)
        while True:
            h = await asyncio.wait_for(r.readline(), 5)
            if h in (b"\r\n", b"\n", b""):
                break
        parts = req.split()
        if len(parts) < 2:
            return
        method = parts[0].decode()
        path, _, query = parts[1].decode().partition("?")
        q = parse_query(query)

        if path == "/":
            await send(w, INDEX, "text/html; charset=utf-8")
        elif path == "/api/state":
            await send(w, json.dumps(tank.state()))
        elif method == "POST" and path.startswith("/api/cal/"):
            err = await tank.api_cal(path[9:], q)
            await send(w, json.dumps({"ok": err is None, "err": err}))
        else:
            await send(w, "Not found", "text/plain", "404 Not Found")
    except Exception:
        pass
    finally:
        try:
            w.close()
            await w.wait_closed()
        except Exception:
            pass


def wifi_connect():
    try:
        network.hostname(HOSTNAME)
    except Exception:
        pass
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    try:
        wlan.config(pm=wlan.PM_NONE)  # no power-save: snappier web page
    except Exception:
        pass
    if TX_POWER:
        try:
            wlan.config(txpower=TX_POWER)
        except Exception:
            pass
    wlan.connect(SSID, PASSWORD)
    t0 = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > 20000:
            break
        time.sleep_ms(250)
    if wlan.isconnected():
        print("RO tank at http://%s/" % wlan.ifconfig()[0])
    else:
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=AP_SSID, password=AP_PASSWORD, authmode=3)
        print("WiFi failed. Join '%s' and open http://192.168.4.1/" % AP_SSID)
    return wlan


async def wifi_watchdog(wlan):
    while True:
        await asyncio.sleep(15)
        if not wlan.isconnected():
            try:
                wlan.disconnect()
                wlan.connect(SSID, PASSWORD)
            except Exception:
                pass


async def main():
    wlan = wifi_connect()
    asyncio.create_task(tank.run())
    asyncio.create_task(wifi_watchdog(wlan))
    await asyncio.start_server(handle, "0.0.0.0", 80)
    gc.collect()
    while True:
        await asyncio.sleep(3600)


asyncio.run(main())