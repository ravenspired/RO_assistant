# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

# main.py - ESP32-C3 WiFi kitchen scale (HX711 + 20 kg load cell), MicroPython
# Serves a web UI on the board's IP: live weight, tare, g/oz/lb:oz, 3-point calibration.
#
# Wiring (ESP32-C3 Super Mini):
#   HX711 VCC -> 3V3   (keep it at 3.3 V so DOUT is safe for the C3)
#   HX711 GND -> GND
#   HX711 DT  -> GPIO4
#   HX711 SCK -> GPIO5
#   Load cell: red=E+, black=E-, white=A-, green=A+  (wrong polarity is fine,
#   calibration handles a reversed sign)

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
HOSTNAME = "kitchenscale"
TX_POWER = 8.5          # dBm, helps flaky Super Mini antennas. None = default
AP_SSID = "KitchenScale"  # fallback hotspot if WiFi fails at boot (IP 192.168.4.1)
AP_PASSWORD = "kitchenscale"

HX_DOUT = 4
HX_SCK = 5

AVG_N = 10              # averaging window (~1 s at the HX711's 10 SPS)
JUMP_G = 3.0            # sudden change bigger than this flushes the filter
HYST_G = 0.07           # displayed value only moves when it changes this much
STABLE_G = 0.25         # "stable" if first/second half of window agree within this
CAPACITY_G = 20000
DEFAULT_CPG = 215.0     # rough counts/gram for a 2 mV/V cell, used before calibration
CAPTURE_N = 30          # samples per calibration capture (~3 s)
DRIFT_G = 2.0           # reject a capture if the reading moved more than this
MIN_SPAN = 1000         # min raw-count difference between calibration points
CAL_FILE = "cal.json"
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


class Scale:
    def __init__(self):
        self.hx = HX711(HX_DOUT, HX_SCK)
        self.buf = []
        self.seq = 0
        self.last_raw = 0
        self.last_ms = time.ticks_ms()
        self.base8 = None
        self.raw8 = 0
        self.g_abs = 0.0
        self.tare_g = 0.0
        self.disp = 0.0
        self.stable = False
        self.cal = {}
        self.pts = []
        self.load_cal()

    # ---------------- calibration storage ----------------
    def load_cal(self):
        try:
            with open(CAL_FILE) as f:
                self.cal = json.load(f)
        except Exception:
            self.cal = {}
        self.rebuild()

    def save_cal(self):
        try:
            if self.cal:
                with open(CAL_FILE, "w") as f:
                    json.dump(self.cal, f)
            else:
                import os
                os.remove(CAL_FILE)
        except OSError:
            pass

    def rebuild(self):
        c = self.cal
        pts = []
        if "zero" in c and ("light" in c or "heavy" in c):
            pts.append((c["zero"], 0.0))
            for k in ("light", "heavy"):
                if k in c:
                    pts.append((c[k][0], c[k][1]))
            pts.sort()
        self.pts = pts
        self.buf = []
        self.tare_g = 0.0

    def clear_cal(self):
        self.cal = {}
        self.save_cal()
        self.rebuild()

    def validate(self, c):
        if "zero" not in c:
            return "Capture ZERO first"
        pts = [(0.0, c["zero"])]
        for k in ("light", "heavy"):
            if k in c:
                pts.append((c[k][1], c[k][0]))
        pts.sort()
        sign = 0
        for (g0, r0), (g1, r1) in zip(pts, pts[1:]):
            if g1 - g0 < 1.0:
                return "Known weights must be different"
            dr = r1 - r0
            if abs(dr) < MIN_SPAN * 8:
                return "Reading barely changed - is the weight on the platform?"
            s = 1 if dr > 0 else -1
            if sign and s != sign:
                return "Readings don't follow the entered weights - check them"
            sign = s
        return None

    # ---------------- conversion ----------------
    def to_g(self, r8):
        pts = self.pts
        if len(pts) < 2:
            if self.base8 is None:
                return 0.0
            return (r8 - self.base8) / (8.0 * DEFAULT_CPG)
        i = 0
        last = len(pts) - 2
        while i < last and r8 > pts[i + 1][0]:
            i += 1
        ra, ga = pts[i]
        rb, gb = pts[i + 1]
        slope = (gb - ga) / (rb - ra)
        return ga + (r8 - ra) * slope

    # ---------------- sampling ----------------
    def ingest(self, raw):
        self.seq += 1
        self.last_raw = raw
        self.last_ms = time.ticks_ms()
        r8 = raw * 8
        if self.base8 is None:
            self.base8 = r8
        g_inst = self.to_g(r8)
        if self.buf and abs(g_inst - self.g_abs) > JUMP_G:
            self.buf = []
        self.buf.append(raw)
        if len(self.buf) > AVG_N:
            self.buf.pop(0)
        self.raw8 = mean8(self.buf)
        self.g_abs = self.to_g(self.raw8)

        n = len(self.buf)
        if n >= AVG_N:
            h = n // 2
            a = self.to_g(mean8(self.buf[:h]))
            b = self.to_g(mean8(self.buf[h:]))
            self.stable = abs(a - b) < STABLE_G
        else:
            self.stable = False

        net = self.g_abs - self.tare_g
        if abs(net - self.disp) >= HYST_G:
            self.disp = round(net, 1) + 0.0

    async def run(self):
        hx = self.hx
        while True:
            if hx.ready():
                try:
                    self.ingest(hx.read())
                except Exception:
                    pass
            await asyncio.sleep_ms(5)

    # ---------------- actions ----------------
    def tare(self):
        self.tare_g = self.g_abs
        self.disp = 0.0

    def untare(self):
        self.tare_g = 0.0

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
        vals = vals[3:]  # discard the first few after the button press
        h = len(vals) // 3
        if abs(mean8(vals[:h]) - mean8(vals[-h:])) > DRIFT_G * DEFAULT_CPG * 8:
            return None, "Reading was drifting - hold still and retry"
        return mean8(vals), None

    async def cal_step(self, step, grams):
        if step not in ("zero", "light", "heavy"):
            return "Bad step"
        if step != "zero":
            if "zero" not in self.cal:
                return "Capture ZERO first"
            if grams <= 0:
                return "Enter the known weight"
        raw8, err = await self.capture()
        if err:
            return err
        new = dict(self.cal)
        new[step] = raw8 if step == "zero" else [raw8, grams]
        err = self.validate(new)
        if err:
            return err
        self.cal = new
        self.save_cal()
        self.rebuild()
        return None

    def state(self):
        c = self.cal
        err = None
        if time.ticks_diff(time.ticks_ms(), self.last_ms) > 2000:
            err = "HX711 not responding - check wiring"
        over = self.g_abs > CAPACITY_G * 1.05 or self.last_raw in (8388607, -8388608)
        return {
            "g": self.disp,
            "stable": self.stable,
            "raw": self.raw8 // 8,
            "tare": round(self.tare_g, 1),
            "calibrated": len(self.pts) >= 2,
            "over": over,
            "err": err,
            "c": {
                "zero": c["zero"] // 8 if "zero" in c else None,
                "light": [c["light"][0] // 8, c["light"][1]] if "light" in c else None,
                "heavy": [c["heavy"][0] // 8, c["heavy"][1]] if "heavy" in c else None,
            },
        }


INDEX = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Kitchen Scale</title>
<style>
:root{color-scheme:dark}
body{margin:0;padding:16px;font-family:system-ui,sans-serif;background:#111;color:#eee;display:flex;flex-direction:column;align-items:center}
.card{background:#1c1c1e;border-radius:16px;padding:18px;width:100%;max-width:440px;margin-bottom:14px;box-sizing:border-box}
#w{font-size:clamp(40px,14vw,68px);font-weight:600;text-align:center;font-variant-numeric:tabular-nums;margin:6px 0}
#sub{text-align:center;color:#999;font-size:14px;min-height:20px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#555;margin-right:6px}
.dot.ok{background:#3ddc84}
.row{display:flex;gap:8px;margin-top:12px}
button{flex:1;padding:14px;border:0;border-radius:12px;background:#2c2c2e;color:#eee;font-size:16px}
button.p{background:#0a84ff;color:#fff}
button:disabled{opacity:.5}
.u button{padding:8px;font-size:14px}
.u button.on{background:#0a84ff}
#banner{display:none;background:#5a3b00;color:#ffd48a;border-radius:10px;padding:10px;font-size:14px;margin-bottom:10px;text-align:center}
#banner.bad{background:#5a1010;color:#ffb0b0}
h3{margin:0 0 6px}
.step{border-top:1px solid #333;padding:12px 0}
.step small{color:#999;display:block;margin:2px 0 8px}
.step .st{font-size:13px;color:#3ddc84}
.in{display:flex;gap:8px}
input,select{background:#2c2c2e;border:0;border-radius:10px;color:#eee;padding:12px;font-size:16px;min-width:0}
input{flex:1}
#msg{font-size:14px;min-height:20px;color:#ffd48a;margin-top:8px}
</style></head><body>
<div class="card">
<div id="banner"></div>
<div id="w">--</div>
<div id="sub"><span class="dot" id="dot"></span><span id="subt"></span></div>
<div class="row"><button class="p" onclick="act('/api/tare')">TARE</button>
<button onclick="act('/api/tare/clear')">Clear tare</button></div>
<div class="row u"><button id="u_g" class="on" onclick="setU('g')">g</button>
<button id="u_oz" onclick="setU('oz')">oz</button>
<button id="u_lb" onclick="setU('lb')">lb:oz</button></div>
</div>

<div class="card"><details><summary><b>Calibration</b></summary>
<div class="step"><b>1. Zero</b>
<small>Empty platform (leave any bowl/tray you always use on it).</small>
<button id="b_zero" onclick="cap('zero')">Capture zero</button>
<div class="st" id="s_zero"></div></div>
<div class="step"><b>2. Light weight</b>
<small>Something light you know exactly, e.g. 100-500 g.</small>
<div class="in"><input id="g_light" inputmode="decimal" placeholder="known weight">
<select id="k_light"><option>g</option><option>kg</option></select></div>
<div class="row"><button id="b_light" onclick="cap('light')">Capture light</button></div>
<div class="st" id="s_light"></div></div>
<div class="step"><b>3. Heavy weight</b>
<small>Something heavy you know exactly, e.g. 5-15 kg (1 L of water = 1000 g).</small>
<div class="in"><input id="g_heavy" inputmode="decimal" placeholder="known weight">
<select id="k_heavy"><option>g</option><option>kg</option></select></div>
<div class="row"><button id="b_heavy" onclick="cap('heavy')">Capture heavy</button></div>
<div class="st" id="s_heavy"></div></div>
<div id="msg"></div>
<div class="row"><button onclick="if(confirm('Clear calibration?'))act('/api/cal/clear')">Clear calibration</button></div>
</details></div>

<script>
const $=id=>document.getElementById(id);
let unit=localStorage.getItem('u')||'g';
function setU(u){unit=u;localStorage.setItem('u',u);for(const k of ['g','oz','lb'])$('u_'+k).className=(k==u?'on':'')}
setU(unit);
function fmt(g){
  if(Math.abs(g)<0.05)g=0;
  if(unit==='g')return g.toFixed(1)+' g';
  const oz=g/28.349523125;
  if(unit==='oz')return oz.toFixed(2)+' oz';
  const s=oz<0?'-':'',a=Math.abs(oz),lb=Math.floor(a/16);
  return s+lb+' lb '+(a-lb*16).toFixed(1)+' oz';
}
let busy=false;
function render(d){
  const b=$('banner');
  if(d.err){b.className='bad';b.style.display='block';b.textContent=d.err}
  else if(d.over){b.className='bad';b.style.display='block';b.textContent='Overload - remove weight (max 20 kg)'}
  else if(!d.calibrated){b.className='';b.style.display='block';b.textContent='Not calibrated - readings are approximate'}
  else b.style.display='none';
  $('w').textContent=d.err?'--':fmt(d.g);
  $('dot').className='dot'+(d.stable?' ok':'');
  $('subt').textContent=(d.stable?'stable':'settling')+(d.tare?'  |  tare '+d.tare.toFixed(1)+' g':'')+'  |  raw '+d.raw;
  const c=d.c;
  $('s_zero').textContent=c.zero!=null?'✓ captured (raw '+c.zero+')':'';
  $('s_light').textContent=c.light?'✓ '+c.light[1]+' g (raw '+c.light[0]+')':'';
  $('s_heavy').textContent=c.heavy?'✓ '+c.heavy[1]+' g (raw '+c.heavy[0]+')':'';
}
async function poll(){
  try{const r=await fetch('/api/weight',{cache:'no-store'});render(await r.json())}
  catch(e){$('subt').textContent='connection lost...'}
  setTimeout(poll,250);
}
async function act(p){try{await fetch(p,{method:'POST'})}catch(e){}}
async function cap(step){
  if(busy)return;
  let g=0;
  if(step!=='zero'){
    g=parseFloat($('g_'+step).value);
    if(!(g>0)){$('msg').textContent='Enter the known weight first';return}
    if($('k_'+step).value==='kg')g*=1000;
  }
  busy=true;
  for(const s of ['zero','light','heavy'])$('b_'+s).disabled=true;
  $('msg').textContent='Capturing '+step+'... keep everything still (~4 s)';
  try{
    const r=await fetch('/api/cal/capture?step='+step+'&g='+g,{method:'POST'});
    const d=await r.json();
    $('msg').textContent=d.ok?'Captured '+step+' ✓':'Error: '+d.err;
  }catch(e){$('msg').textContent='Request failed'}
  for(const s of ['zero','light','heavy'])$('b_'+s).disabled=false;
  busy=false;
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


scale = Scale()


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
        elif path == "/api/weight":
            await send(w, json.dumps(scale.state()))
        elif method == "POST" and path == "/api/tare":
            scale.tare()
            await send(w, '{"ok":true}')
        elif method == "POST" and path == "/api/tare/clear":
            scale.untare()
            await send(w, '{"ok":true}')
        elif method == "POST" and path == "/api/cal/capture":
            try:
                g = float(q.get("g", "0"))
            except ValueError:
                g = 0.0
            err = await scale.cal_step(q.get("step", ""), g)
            await send(w, json.dumps({"ok": err is None, "err": err}))
        elif method == "POST" and path == "/api/cal/clear":
            scale.clear_cal()
            await send(w, '{"ok":true}')
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
        print("Scale at http://%s/" % wlan.ifconfig()[0])
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
    asyncio.create_task(scale.run())
    asyncio.create_task(wifi_watchdog(wlan))
    await asyncio.start_server(handle, "0.0.0.0", 80)
    gc.collect()
    while True:
        await asyncio.sleep(3600)


asyncio.run(main())
