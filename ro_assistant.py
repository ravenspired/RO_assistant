"""
RO tank scale  -  Raspberry Pi Pico W (RP2040), MicroPython
HX711 + 3 parallel load cells, live WiFi dashboard.

Files on the Pico:
    main.py       this file (runs on boot)
    index.html    the dashboard page
Files the Pico creates:
    calib.json       calibration (empty point, scale, capacity, refill curve)
    cal_wip.json     calibration wizard progress (survives power cycles)
    refill_log.csv   fill rate vs fullness, logged during calibration
    pours.csv        history of every pour
<<<<<<< Updated upstream
    settings.json    dispense presets and the valve close lead (+ auto-tune flag)
=======
    settings.json    dispense presets, the learned valve lag, the full-tank correction
>>>>>>> Stashed changes
    stats.json       hourly + daily water-use totals for the Stats page
    drain.json       learned drain rate vs fullness (drives the dispense countdown)
    snaps.csv        log of every full-tank reset (how far the reading had drifted)

Wiring is the same as your test script: DT -> GP14, SCK -> GP15.
Dispense solenoid valve: GP16 (normally open; closed only at the end of a dispense, see VALVE_* below).
"""
import gc
import json
import os
import time

import network
import uasyncio as asyncio
from machine import Pin, WDT, disable_irq, enable_irq

try:
    import ntptime
except ImportError:
    ntptime = None

# ============================ CONFIG ============================
WIFI_SSID = "YOUR_WIFI_NAME"
WIFI_PASS = "YOUR_WIFI_PASSWORD"
HOSTNAME = "ro-tank"

DT_PIN = 14
SCK_PIN = 15
GAIN_PULSES = 1            # 1 = ch A gain 128

USE_WDT = False            # set True once everything works (resets the Pico if it hangs)

N_BINS = 40                # resolution of the fill-rate-vs-fullness curve
FULL_FRAC = 0.99           # at/above this fraction the tank counts as "full"

POUR_END_MS = 10000        # a pour ends after this long with no draw
OUT_START = 5.0            # ml/s of outflow needed to start a pour
OUT_KEEP = 2.5             # ml/s of outflow that keeps a pour alive
SLOPE_START = 2.0          # g/s the weight must actually be dropping to start a pour
PRE_ROLL_MS = 3000         # back-date a pour start to catch the first second of flow
SETTLE_MS = 3000           # keep counting this long after the last draw (scale settling)
MIN_POUR_ML = 15           # ignore "pours" smaller than this (bumps, noise)
ONSET_G = 3.0              # grams the weight must fall below its resting value to mark the start of a draw
POUR_MAX_MS = 1200000      # safety: a manual pour can't stay "active" longer than this

# ---- full-tank reset (thermal drift of the scale reading) ----
SNAP_ENABLE = True
SNAP_MIN_FRAC = 0.85       # the reading must be at least this full to count as "stuck near full"
SNAP_MAX_FRAC = 1.25       # ...and no more than this (further off than that means something else is wrong)
SNAP_HOLD_S = 1800         # no pour and no real rise for this long -> the tank is full: reset to 100 %
SNAP_RISE_ML = 40          # a rise bigger than this above the lowest point is real refilling: restart the timer
SNAP_MIN_ML = 30           # corrections smaller than this are ignored (noise), so a healthy full tank stays quiet
STALE_S = 90               # quiet this long with no fill-rate measurement -> re-sync the estimators

CAP_MS = 8000              # window used when recording full/empty weights
DRIFT_MAX = 6000           # max counts of drift allowed while recording a weight
MIN_CAL_COUNTS = 20000     # full and empty must differ by at least this many counts
LOG_PERIOD_MS = 30000      # refill log interval during calibration

# ---- dispense valve (normally-open solenoid on GP16) ----
VALVE_PIN = 16
VALVE_OPEN_LEVEL = 0       # GPIO level that lets water through. NO valve: coil off = open.
                           # Flip to 1 if your driver stage inverts. Idle = valve OPEN (coil off),
                           # so the sink works normally. It is only closed during a dispense.
VALVE_LAG_S = 0.25         # valve close lead, in seconds. The valve is told to close when
                           #   dispensed_ml + current_flow_ml_s * VALVE_LAG_S >= target_ml
                           # so fast flow closes earlier and slow flow closes later.
                           # Starting value only: change it on the touchscreen (Settings) or via
                           # /api/settings?lag=0.25. Auto-tuned after each dispense unless disabled.
VALVE_LAG_MAX = 1.5        # largest lead (s) that can be set or learned
DISP_MAX_ML = 9000
DISP_STALL_MS = 15000      # valve open this long without ~5 ml of progress -> close
DISP_STALL_ML = 5
DISP_MAX_MS = 900000       # hard cap on one dispense
DISP_SETTLE_MS = 2500      # keep measuring this long after the valve closes
DISP_ARM_MS = 120000       # "start pouring" with no flow for this long cancels itself
DISP_LEARN_MS = 1500       # ignore the first part of a run (flow still ramping up) for timing and learning
DISP_FALSE_ML = 10         # a run that has moved less than this after 3 s was a false trigger
OUT_BINS = 20              # resolution of the drain-rate-vs-fullness curve (5 % steps)

# ---- usage statistics ----
STATS_UTC_OFFSET_H = -6    # where "midnight" falls for the daily totals (-6 = US Central standard time)
STATS_HOURS = 168          # hourly buckets kept (drives the 24 h and 7 day figures)
STATS_DAYS = 365           # daily buckets kept (drives the 30 day and year figures)
CLOCK_MIN = 1000000000     # epoch values below this mean "clock not set"
# ================================================================

CALIB_FILE = "calib.json"
WIP_FILE = "cal_wip.json"
LOG_FILE = "refill_log.csv"
POURS_FILE = "pours.csv"
SETTINGS_FILE = "settings.json"
STATS_FILE = "stats.json"
DRAIN_FILE = "drain.json"
SNAP_FILE = "snaps.csv"
EPOCH_OFFSET = 946684800   # MicroPython epoch is 2000-01-01

ntp_ok = False

# Valve pin is created at import and idles OPEN (coil off): normal sink operation.
valve = Pin(VALVE_PIN, Pin.OUT, value=VALVE_OPEN_LEVEL)


def valve_set(is_open):
    valve.value(VALVE_OPEN_LEVEL if is_open else 1 - VALVE_OPEN_LEVEL)


# ------------------------- small helpers -------------------------
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def epoch():
    return (time.time() + EPOCH_OFFSET) if ntp_ok else 0


_mono = 0
_last_ticks = time.ticks_ms()


def mono():
    """Monotonic milliseconds that never wrap (call at least every few seconds)."""
    global _mono, _last_ticks
    n = time.ticks_ms()
    _mono += time.ticks_diff(n, _last_ticks)
    _last_ticks = n
    return _mono


def load_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return default


def save_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    try:
        os.rename(tmp, path)
    except OSError:
        try:
            os.remove(path)
        except OSError:
            pass
        os.rename(tmp, path)


def remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


def exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def lsq(ts, vs, i0, i1):
    """Least-squares slope of vs against ts (ms) over [i0, i1), in units per second."""
    n = i1 - i0
    t0 = ts[i0]
    y0 = vs[i0]
    sx = 0.0
    sy = 0.0
    for i in range(i0, i1):
        sx += (ts[i] - t0) * 0.001
        sy += vs[i] - y0
    mx = sx / n
    my = sy / n
    num = 0.0
    den = 0.0
    for i in range(i0, i1):
        dx = (ts[i] - t0) * 0.001 - mx
        num += dx * ((vs[i] - y0) - my)
        den += dx * dx
    if den < 1e-6:
        return None
    return num / den


class Buf:
    """Time-windowed buffer of (t_ms, value)."""

    def __init__(self, max_age_ms):
        self.t = []
        self.v = []
        self.max_age = max_age_ms

    def add(self, t, v):
        self.t.append(t)
        self.v.append(v)
        lim = t - self.max_age
        k = 0
        n = len(self.t)
        while k < n and self.t[k] < lim:
            k += 1
        if k:
            del self.t[:k]
            del self.v[:k]

    def slope(self, now, win_ms, since=None, minpts=5):
        lim = now - win_ms
        if since is not None and since > lim:
            lim = since
        ts = self.t
        n = len(ts)
        i = 0
        while i < n and ts[i] < lim:
            i += 1
        if n - i < minpts:
            return None
        return lsq(ts, self.v, i, n)


def curve_at(c, f):
    """Fill rate (ml/s) at fullness f (0..1), linearly interpolated between bin centres."""
    n = len(c)
    p = f * n - 0.5
    if p <= 0:
        return c[0]
    if p >= n - 1:
        return c[n - 1]
    i = int(p)
    fr = p - i
    return c[i] * (1.0 - fr) + c[i + 1] * fr


def build_curve():
    """Turn refill_log.csv into a rate-vs-fullness table. Returns (curve, rows, fmax)."""
    sums = [0.0] * N_BINS
    cnt = [0] * N_BINS
    rows = 0
    fmax = 0.0
    try:
        with open(LOG_FILE) as fh:
            fh.readline()
            for line in fh:
                p = line.split(",")
                if len(p) < 5:
                    continue
                try:
                    f = float(p[1])
                    r = float(p[3])
                except ValueError:
                    continue
                i = int(clamp(f * N_BINS, 0, N_BINS - 1))
                sums[i] += r
                cnt[i] += 1
                rows += 1
                if f > fmax:
                    fmax = f
    except OSError:
        return None, 0, 0.0
    known = [i for i in range(N_BINS) if cnt[i]]
    if not known:
        return None, rows, fmax
    c = [None] * N_BINS
    for i in known:
        c[i] = sums[i] / cnt[i]
    for i in range(N_BINS):
        if c[i] is not None:
            continue
        a = None
        b = None
        for k in known:
            if k < i:
                a = k
            elif k > i and b is None:
                b = k
        if a is None:
            c[i] = c[b]
        elif b is None:
            c[i] = c[a]
        else:
            c[i] = c[a] + (c[b] - c[a]) * (i - a) / (b - a)
    sm = []
    for i in range(N_BINS):
        lo = max(0, i - 1)
        hi = min(N_BINS - 1, i + 1)
        seg = c[lo:hi + 1]
        sm.append(max(0.0, sum(seg) / len(seg)))
    return [round(x, 4) for x in sm], rows, fmax


def parse_qs(qs):
    d = {}
    for kv in qs.split("&"):
        if "=" in kv:
            k, v = kv.split("=", 1)
            d[k] = v
    return d


# --------------------------- HX711 ---------------------------
class HX711:
    def __init__(self, dt, sck, gain_pulses=1):
        self.dt = Pin(dt, Pin.IN, Pin.PULL_UP)
        self.sck = Pin(sck, Pin.OUT, value=0)
        self.gp = gain_pulses

    def ready(self):
        return self.dt.value() == 0

    def read(self):
        """One signed 24-bit reading. Call only when ready()."""
        val = 0
        irq = disable_irq()  # SCK high > 60 us would power the chip down
        for _ in range(24):
            self.sck.value(1)
            self.sck.value(0)
            val = (val << 1) | self.dt.value()
        for _ in range(self.gp):
            self.sck.value(1)
            self.sck.value(0)
        enable_irq(irq)
        if val & 0x800000:
            val -= 0x1000000
        return val


# --------------------------- the tank model ---------------------------
class Tank:
    def __init__(self):
        self.cal = None              # {'empty','scale','cap','curve','at'}
        self.wip = {"step": "idle"}  # calibration wizard progress
        self.ip = ""
        self.raws = Buf(10000)
        self.gbuf = Buf(6000)        # grams, ~10 Hz, short
        self.hist = Buf(905000)      # grams, 1 Hz, 15 min
        self.r3 = []
        self.med = None
        self.g = None
        self.last_sample_t = None
        self.sensor_ok = False
        self.sat = False
        self.prev_t = None
        self.corr = 1.0              # live inflow / calibrated-curve inflow
        self.in_meas = None
        self.meas_win = 0
        self.slope2 = 0.0
        self.out_ema = 0.0
        self.draw_cnt = 0
        self.last_draw_t = None
        self.pour = None
        self.pours = []
        self.last_pour = None
        self.eta_s = None
        self.eta_src = ""
        self.state = "idle"
        self.last_log_t = 0
        self.log_n = 0
        self.settings = {"presets": [200, 1000, 3700, 2000, 0, 0, 0, 0], "lag": VALVE_LAG_S, "learn": True}
        self.disp = None             # dispense job, see disp_action()
        self.stats_h = {}            # hour index -> [ml, pours]
        self.stats_d = {}            # day index  -> [ml, pours]
        self.stats_dirty = False
        self.off = 0.0               # grams added to every reading by full-tank resets
        self.off_dirty = False
        self.off_saved_t = -10 ** 9
        self.plat_t0 = None          # start of the current "stuck near full" window
        self.plat_min = 0.0
        self.full_latch = False      # set by a full-tank reset, cleared by the next draw
        self.meas_t = 0              # last time a fill-rate measurement existed (or a pour was running)
        self.resyncs = 0
        self.ev = {"n": 0, "k": "", "ml": 0}   # last notable event, for a dashboard toast
        self.rep_id = 0
        self.last_rep = None         # report for the most recent finished pour (manual or auto)
        self.out_m = [0.0] * OUT_BINS  # learned drain rate (ml/s) per fullness bin
        self.out_n = [0] * OUT_BINS    # how many pours have contributed to each bin
        self.out_curve = None          # out_m with the gaps interpolated

    # ---------- persistence ----------
    def load(self):
        self.cal = load_json(CALIB_FILE, None)
        wip = load_json(WIP_FILE, None)
        if wip:
            self.wip = wip
        st = load_json(SETTINGS_FILE, None)
        if isinstance(st, dict):
            try:
                p = st.get("presets")
                if isinstance(p, list) and len(p) == 8:
                    self.settings["presets"] = [int(clamp(int(x), 0, DISP_MAX_ML)) for x in p]
                if "lag" in st:
<<<<<<< Updated upstream
                    self.settings["lag"] = clamp(float(st["lag"]), 0.0, VALVE_LAG_MAX)
                if "learn" in st:
                    self.settings["learn"] = bool(st["learn"])
=======
                    self.settings["lag"] = clamp(float(st["lag"]), 0.0, 1.5)
                if "off" in st:
                    self.off = clamp(float(st["off"]), -5000.0, 5000.0)
>>>>>>> Stashed changes
            except Exception as e:
                print("settings load:", e)
        dr = load_json(DRAIN_FILE, None)
        if isinstance(dr, dict):
            try:
                m = dr.get("m")
                n = dr.get("n")
                if isinstance(m, list) and isinstance(n, list) and len(m) == OUT_BINS and len(n) == OUT_BINS:
                    self.out_m = [float(x) for x in m]
                    self.out_n = [int(x) for x in n]
            except Exception as e:
                print("drain load:", e)
        self._out_rebuild()
        self.meas_t = mono()
        self._stats_load()
        if self.wip.get("step") == "refill":
            self.log_n = max(0, self._count_lines(LOG_FILE) - 1)
        if not exists(POURS_FILE):
            with open(POURS_FILE, "w") as f:
                f.write("epoch,ml,duration_s,start_pct,end_pct\n")
        else:
            try:
                with open(POURS_FILE) as f:
                    f.readline()
                    for line in f:
                        p = line.strip().split(",")
                        if len(p) == 5:
                            self.pours.append([int(p[0]), int(p[1]), float(p[2]), float(p[3]), float(p[4])])
                            if len(self.pours) > 20:
                                self.pours.pop(0)
                if self.pours:
                    self.last_pour = self.pours[-1]
            except Exception as e:
                print("pours load:", e)

    # ---------- usage statistics ----------
    def _stats_load(self):
        st = load_json(STATS_FILE, None)
        ok = False
        if isinstance(st, dict) and st.get("off") == STATS_UTC_OFFSET_H:
            try:
                for k, ml, n in st.get("h", []):
                    self.stats_h[int(k)] = [int(ml), int(n)]
                for k, ml, n in st.get("d", []):
                    self.stats_d[int(k)] = [int(ml), int(n)]
                ok = True
            except Exception as e:
                print("stats load:", e)
                self.stats_h = {}
                self.stats_d = {}
        if not ok:
            self._stats_rebuild()

    def _stats_rebuild(self):
        """First boot with stats (or the UTC offset changed): rebuild from the pour history."""
        self.stats_h = {}
        self.stats_d = {}
        newest = 0
        n = 0
        for path in ("pours_old.csv", POURS_FILE):
            try:
                with open(path) as f:
                    f.readline()
                    for line in f:
                        p = line.split(",")
                        try:
                            ep = int(p[0])
                            ml = int(p[1])
                        except (ValueError, IndexError):
                            continue
                        if ep < CLOCK_MIN:
                            continue
                        if ep > newest:
                            newest = ep
                        self._stats_put(ep, ml)
                        n += 1
                        if n % 64 == 0:
                            self._stats_prune(newest)
            except OSError:
                pass
        if newest:
            self._stats_prune(newest)
        self.stats_dirty = True

    def _stats_put(self, ep, ml):
        h = ep // 3600
        d = (ep + STATS_UTC_OFFSET_H * 3600) // 86400
        for tbl, k in ((self.stats_h, h), (self.stats_d, d)):
            a = tbl.get(k)
            if a:
                a[0] += ml
                a[1] += 1
            else:
                tbl[k] = [ml, 1]

    def _stats_prune(self, ep):
        nh = ep // 3600
        nd = (ep + STATS_UTC_OFFSET_H * 3600) // 86400
        for k in [k for k in self.stats_h if nh - k >= STATS_HOURS]:
            del self.stats_h[k]
        for k in [k for k in self.stats_d if nd - k >= STATS_DAYS]:
            del self.stats_d[k]

    def _stats_add(self, ep, ml):
        if ep < CLOCK_MIN:
            return          # clock not set yet: can't place this pour in time
        self._stats_put(ep, ml)
        self._stats_prune(ep)
        self.stats_dirty = True

    def _stats_save(self):
        self.stats_dirty = False
        try:
            save_json(STATS_FILE, {
                "off": STATS_UTC_OFFSET_H,
                "h": [[k, v[0], v[1]] for k, v in self.stats_h.items()],
                "d": [[k, v[0], v[1]] for k, v in self.stats_d.items()]})
        except Exception as e:
            print("stats save:", e)

    def stats_snapshot(self):
        ep = epoch()
        if ep < CLOCK_MIN:
            return {"ok": False, "msg": "Clock not set yet. Waiting for network time (NTP)."}
        nh = ep // 3600
        nd = (ep + STATS_UTC_OFFSET_H * 3600) // 86400
        hrs = [0] * 24
        w1 = [0, 0]
        w7 = [0, 0]
        for k, v in self.stats_h.items():
            a = nh - k
            if 0 <= a < STATS_HOURS:
                w7[0] += v[0]
                w7[1] += v[1]
                if a < 24:
                    w1[0] += v[0]
                    w1[1] += v[1]
                    hrs[23 - a] = v[0]
        days = [0] * STATS_DAYS
        w30 = [0, 0]
        w365 = [0, 0]
        for k, v in self.stats_d.items():
            a = nd - k
            if 0 <= a < STATS_DAYS:
                days[STATS_DAYS - 1 - a] = v[0]
                w365[0] += v[0]
                w365[1] += v[1]
                if a < 30:
                    w30[0] += v[0]
                    w30[1] += v[1]
        return {"ok": True, "ep": ep,
                "w": {"d1": w1, "d7": w7, "d30": w30, "d365": w365},
                "hours": hrs, "days": days}

    def _count_lines(self, path):
        n = 0
        try:
            with open(path) as f:
                for _ in f:
                    n += 1
        except OSError:
            pass
        return n

    def _save_wip(self):
        if self.wip.get("step", "idle") == "idle":
            remove(WIP_FILE)
        else:
            save_json(WIP_FILE, self.wip)

    def reset_runtime(self):
        self.gbuf = Buf(6000)
        self.hist = Buf(905000)
        self.g = None
        self.pour = None
        self.draw_cnt = 0
        self.last_draw_t = None
        self.prev_t = None
        self.in_meas = None
        self.corr = 1.0
        self.out_ema = 0.0
        self.eta_s = None
        self.off = 0.0               # a fresh calibration starts with no drift correction
        self.off_dirty = True
        self.plat_t0 = None
        self.full_latch = False
        self.meas_t = mono()

    def _save_settings(self):
        self.settings["off"] = round(self.off, 1)
        self.off_dirty = False
        self.off_saved_t = mono()
        save_json(SETTINGS_FILE, self.settings)

    # ---------- sensor input (10 Hz) ----------
    def on_sample(self, raw):
        t = mono()
        self.last_sample_t = t
        self.sensor_ok = True
        self.sat = raw >= 8388607 or raw <= -8388608
        self.raws.add(t, raw)
        self.r3.append(raw)
        if len(self.r3) > 3:
            self.r3.pop(0)
        med = sorted(self.r3)[len(self.r3) // 2]
        self.med = med
        if not self.cal:
            return
        g = (med - self.cal["empty"]) * self.cal["scale"] + self.off
        self.g = g
        self.gbuf.add(t, g)
        if self.disp is not None:
            self._disp_sample(t, g)
        else:
            self._pour_logic(t, g)

    def inflow_est(self, f):
        cur = self.cal.get("curve") if self.cal else None
        if not cur:
            return 0.0
        return max(0.0, curve_at(cur, clamp(f, 0.0, 1.0)) * self.corr)

    def _pour_logic(self, t, g):
        cap = self.cal["cap"]
        f = clamp(g / cap, 0.0, 1.2)
        s2 = self.gbuf.slope(t, 2000, None, 5)
        if s2 is None:
            return
        dt = 0.1 if self.prev_t is None else (t - self.prev_t) / 1000.0
        self.prev_t = t
        in_est = self.inflow_est(f)
        out_inst = in_est - s2          # outflow = inflow - dW/dt
        self.slope2 = s2
        a = dt / 0.8 if dt < 0.8 else 1.0
        self.out_ema += a * (max(out_inst, 0.0) - self.out_ema)

        p = self.pour
        if p is None:
            if out_inst > OUT_START and s2 < -SLOPE_START:
                self.draw_cnt += 1
                self.last_draw_t = t
            else:
                self.draw_cnt = 0
            if self.draw_cnt >= 3:
                self._start_pour(t, in_est)
            return

        p["in_int"] += in_est * dt
        vals = self.gbuf.v[-8:]
        ws = sum(vals) / len(vals)
        p["ml"] = max(0.0, p["w0"] - ws + p["in_int"])
        if p["ml"] > p["mx"] + 2.5:          # volume still climbing: remember when it last did
            p["mx"] = p["ml"]
            p["t_rise"] = t
        if out_inst > OUT_KEEP:
            p["last_draw"] = t
            self.last_draw_t = t
        idle = t - p["last_draw"]
        if idle <= SETTLE_MS:
            p["snap"] = p["ml"]
            p["f_end"] = ws / cap
            if self.out_ema > p["peak"]:
                p["peak"] = self.out_ema
        if idle > POUR_END_MS or t - p["t0"] > POUR_MAX_MS:
            self._end_pour(t)

    def _start_pour(self, t, in_est):
        cap = self.cal["cap"]
        t0 = t - PRE_ROLL_MS
        gb = self.gbuf
        i = 0
        while i < len(gb.t) - 3 and gb.t[i] < t0:
            i += 1
        seg = gb.v[i:i + 3]
        w0 = sum(seg) / len(seg)
        t0 = self._onset(i, w0)
        self.pour = {
            "t0": t0, "w0": w0, "in_int": in_est * (t - t0) / 1000.0,
            "last_draw": t, "ml": 0.0, "snap": 0.0, "peak": 0.0,
            "f0": w0 / cap, "f_end": w0 / cap, "mx": 0.0, "t_rise": t0,
        }
        self.draw_cnt = 0
        self.full_latch = False

    def _onset(self, i, w_ref):
        """Time the weight first fell clearly below its resting value: the real start of the draw
        (the pre-roll window starts earlier than that, which would inflate pour times)."""
        gb = self.gbuf
        n = len(gb.t)
        j = i + 3
        while j < n and gb.v[j] > w_ref - ONSET_G:
            j += 1
        if j >= n or j <= i + 3:
            return gb.t[i]
        return gb.t[j - 1]

    def _report(self, rep):
        self.rep_id += 1
        rep["id"] = self.rep_id
        rep["ep"] = epoch()
        self.last_rep = rep
        return rep

    def _end_pour(self, t):
        p = self.pour
        self.pour = None
        self.draw_cnt = 0
        ml = p["snap"]
        # flow time = onset to the moment the volume stopped rising (minus the ~0.35 s the 8-sample
        # smoothing delays it). "last_draw" lags the real stop by over a second, which skewed the average.
        dur = max(0.0, (p["t_rise"] - p["t0"]) / 1000.0 - 0.35)
        if ml < MIN_POUR_ML:
            return
        rec = [epoch(), int(round(ml)), round(dur, 1), round(p["f0"] * 100, 1), round(p["f_end"] * 100, 1)]
        self._log_pour(rec)
        self._report({"kind": "manual", "ml": rec[1], "dur": rec[2],
                      "avg": round(ml / dur, 1) if dur > 0.5 else None,
                      "peak": round(p["peak"], 1), "f0": rec[3], "f_end": rec[4]})

    def _log_pour(self, rec):
        self.pours.append(rec)
        if len(self.pours) > 20:
            self.pours.pop(0)
        self.last_pour = rec
        self._stats_add(rec[0], rec[1])
        try:
            if exists(POURS_FILE) and os.stat(POURS_FILE)[6] > 200000:
                remove("pours_old.csv")
                os.rename(POURS_FILE, "pours_old.csv")
                with open(POURS_FILE, "w") as f:
                    f.write("epoch,ml,duration_s,start_pct,end_pct\n")
            with open(POURS_FILE, "a") as f:
                f.write("%d,%d,%.1f,%.1f,%.1f\n" % tuple(rec))
        except Exception as e:
            print("pour log:", e)

    # ---------- 1 Hz housekeeping ----------
    def tick(self):
        t = mono()
        self.sensor_ok = self.last_sample_t is not None and (t - self.last_sample_t) < 3000
        d = self.disp
        if d is not None:
            st = d["st"]
            if st == "run" and (self.last_sample_t is None or t - self.last_sample_t > 1500):
                self._disp_close(t, "sensor lost")
            elif st == "armed" and t - d["tq"] > DISP_ARM_MS:
                self._disp_end()
            elif st == "settle" and t - d["tc"] > DISP_SETTLE_MS + 2000:
                self._disp_settle(False)    # no samples arrived to finish it, so finish it here
        if self.stats_dirty and self.disp is None:
            self._stats_save()
        if self.off_dirty and self.disp is None and t - self.off_saved_t > 600000:
            try:
                self._save_settings()
            except Exception as e:
                print("settings save:", e)
        if not self.cal or not self.sensor_ok or len(self.gbuf.v) < 5:
            return
        vals = [v for tt, v in zip(self.gbuf.t, self.gbuf.v) if tt >= t - 1000]
        if not vals:
            return
        g1 = sum(vals) / len(vals)
        g1 = self._plateau(t, g1)            # full-tank reset: may shift the reading to exactly 100 %
        self.hist.add(t, g1)
        cap = self.cal["cap"]
        f = clamp(g1 / cap, 0.0, 1.2)

        # measured net inflow while nobody is drawing
        since = (t - self.last_draw_t) if self.last_draw_t is not None else 10 ** 9
        meas = None
        win = 0
        if self.pour is None and since > 9000:
            lo = (self.last_draw_t + 5000) if self.last_draw_t is not None else None
            r = self.hist.slope(t, 60000, lo, 12)
            if r is not None:
                win = 60
                if abs(r) < 0.8:
                    r2 = self.hist.slope(t, 240000, lo, 60)
                    if r2 is not None:
                        r = r2
                        win = 240
                meas = r
        self.in_meas = meas
        self.meas_win = win

        # watchdog: if we have been quiet for a while and STILL have no fill-rate measurement, the
        # estimators are stuck on old data. Throw them away and start clean.
        busy = self.pour is not None or (self.disp is not None and self.disp["st"] != "armed")
        if meas is not None or busy:
            self.meas_t = t
        elif t - self.meas_t > STALE_S * 1000:
            self.resync("stale")
            return

        # keep the live-supply correction up to date. Only learn where the curve is steep enough for a
        # measurement to mean something: near full the real rate is ~0.1 ml/s, the same size as thermal
        # drift, and learning from it used to drag the correction towards zero.
        cur = self.cal.get("curve")
        if meas is not None and cur and f < 0.9:
            cr = curve_at(cur, f)
            if cr > 0.3:
                target = clamp(meas / cr, 0.0, 5.0)
                self.corr += 0.15 * (target - self.corr)

        in_rate, _ = self.in_display(f)
        if f < SNAP_MIN_FRAC:
            self.full_latch = False
        if busy:
            self.state = "pour"
        elif f >= FULL_FRAC or self.full_latch:
            self.state = "full"
        elif in_rate > (0.03 if self.state == "refill" else 0.06):   # hysteresis: drift can't flap it
            self.state = "refill"
        elif meas is not None:
            self.state = "stalled"
        else:
            self.state = "idle"

        # refill countdown
        self.eta_s = None
        self.eta_src = ""
        if self.pour is None and self.state == "refill":
            if cur:
                steps = 40
                df = (FULL_FRAC - f) / steps
                tot = 0.0
                for k in range(steps):
                    r = max(curve_at(cur, f + (k + 0.5) * df) * self.corr, 0.02)
                    tot += df * cap / r
                self.eta_s = tot
                self.eta_src = "curve"
            elif in_rate > 0.03:
                self.eta_s = (FULL_FRAC - f) * cap / in_rate
                self.eta_src = "linear"
            if self.eta_s is not None and (self.eta_s < 0 or self.eta_s > 172800):
                self.eta_s = None            # more than two days is not a countdown, it's a shrug
                self.eta_src = ""

        # calibration refill logging
        if self.wip.get("step") == "refill":
            self._refill_logging(t, f, g1, meas, since)

    # ---------- full-tank reset and re-sync ----------
    def _plateau(self, t, g1):
        """Thermal drift makes a full tank read anywhere from ~90 % to ~110 %. A tank that sits near
        full with no pour and no real rise for SNAP_HOLD_S is full: shift the reading to exactly 100 %.
        A rise bigger than SNAP_RISE_ML above the lowest point is real refilling and restarts the timer,
        so a tank that is genuinely still filling is never reset. Returns the (possibly corrected) g1."""
        cap = self.cal["cap"]
        f = g1 / cap
        quiet = (SNAP_ENABLE and self.pour is None and self.disp is None
                 and self.wip.get("step", "idle") == "idle" and SNAP_MIN_FRAC <= f <= SNAP_MAX_FRAC)
        if (quiet and self.plat_t0 is not None and self.last_draw_t is not None
                and self.last_draw_t >= self.plat_t0):
            quiet = False                    # something was drawn since the window started
        if not quiet:
            self.plat_t0 = None
            return g1
        if self.plat_t0 is None or g1 - self.plat_min > SNAP_RISE_ML:
            self.plat_t0 = t
            self.plat_min = g1
            return g1
        if g1 < self.plat_min:
            self.plat_min = g1
        if t - self.plat_t0 >= SNAP_HOLD_S * 1000:
            return self._snap_full(t, g1, "stuck")
        return g1

    def _snap_full(self, t, g_now, why):
        cap = self.cal["cap"]
        delta = cap - g_now
        if abs(delta) < SNAP_MIN_ML and why != "manual":
            self.plat_t0 = None              # already reads full to within noise: nothing to fix
            self.full_latch = True
            return g_now
        self.off += delta
        gb = self.gbuf.v
        for i in range(len(gb)):
            gb[i] += delta                   # keep the short buffer continuous so no phantom pour appears
        if self.g is not None:
            self.g += delta
        self.plat_t0 = None
        self.full_latch = True
        self.off_dirty = True
        self.resync("full:" + why)
        self._event("full", delta)
        try:
            new = (not exists(SNAP_FILE)) or os.stat(SNAP_FILE)[6] > 20000
            with open(SNAP_FILE, "w" if new else "a") as fh:
                if new:
                    fh.write("epoch,correction_ml,reading_pct,why\n")
                fh.write("%d,%d,%.1f,%s\n" % (epoch(), int(delta), g_now / cap * 100.0, why))
        except Exception as e:
            print("snap log:", e)
        return cap

    def _event(self, kind, ml):
        self.ev = {"n": self.ev["n"] + 1, "k": kind, "ml": int(round(ml))}

    def resync(self, why):
        """Throw away everything the estimators learned from the recent past, so the status, fill rate
        and countdown start from a clean slate. Never touches the valve or a dispense in progress."""
        self.hist = Buf(905000)
        self.in_meas = None
        self.meas_win = 0
        self.corr = 1.0
        self.out_ema = 0.0
        self.slope2 = 0.0
        self.draw_cnt = 0
        self.prev_t = None
        self.last_draw_t = None
        self.eta_s = None
        self.eta_src = ""
        self.meas_t = mono()
        self.resyncs += 1
        print("resync:", why)
        if not why.startswith("full"):
            self._event("resync", 0)

    def in_display(self, f):
        if self.in_meas is not None and self.pour is None:
            return max(0.0, self.in_meas), "meas"
        return self.inflow_est(f), "est"

    def _refill_logging(self, t, f, g1, meas, since):
        if meas is None or self.pour is not None or since < 120000:
            return
        if t - self.last_log_t < LOG_PERIOD_MS:
            return
        self.last_log_t = t
        if meas < 0.02 and f < 0.8:
            return  # valve probably not open yet; don't poison the curve
        try:
            with open(LOG_FILE, "a") as fh:
                fh.write("%d,%.4f,%.1f,%.4f,%d\n" % (epoch(), f, g1, meas, self.meas_win))
            self.log_n += 1
        except Exception as e:
            print("log:", e)
        if f >= 0.9 and self.log_n >= 20:
            lo = (self.last_draw_t + 5000) if self.last_draw_t is not None else None
            r15 = self.hist.slope(t, 900000, lo, 780)
            if r15 is not None and r15 < 0.02:
                ok, msg = self._finish_refill(False)
                print("auto-finish:", msg)

    def _finish_refill(self, force):
        curve, rows, fmax = build_curve()
        if curve is None or ((rows < 20 or fmax < 0.6) and not force):
            return False, "Only %d log rows reaching %d%% so far. Let it refill longer, or press Finish again to save anyway." % (rows, int(fmax * 100))
        if curve is None:
            return False, "No usable refill data was logged."
        self.cal["curve"] = curve
        save_json(CALIB_FILE, self.cal)
        self.wip = {"step": "idle"}
        self._save_wip()
        self.corr = 1.0
        return True, "Calibration saved. Refill curve learned from %d samples." % rows

    # ---------- calibration wizard ----------
    def stability(self):
        t = mono()
        vals = [v for tt, v in zip(self.raws.t, self.raws.v) if tt >= t - CAP_MS]
        n = len(vals)
        if n < 10:
            return {"n": n, "std": 0, "drift": 0, "ok": False, "mean": 0}
        mean = sum(vals) / n
        h = n // 2
        drift = abs(sum(vals[h:]) / (n - h) - sum(vals[:h]) / h)
        std = (sum((v - mean) ** 2 for v in vals) / n) ** 0.5
        return {"n": n, "std": int(std), "drift": int(drift),
                "ok": n >= 30 and drift <= DRIFT_MAX, "mean": int(mean)}

    def _capture(self):
        if not self.sensor_ok:
            return None, "No readings from the HX711. Check wiring."
        s = self.stability()
        if s["n"] < 30:
            return None, "Not enough readings yet. Wait a few seconds and try again."
        if self.sat:
            return None, "HX711 is saturated. Check the load cell wiring."
        if s["drift"] > DRIFT_MAX:
            return None, "Weight is still changing (%d counts of drift). Wait for it to settle, then try again." % s["drift"]
        return s["mean"], "ok"

    def cal_action(self, act, q):
        w = self.wip
        step = w.get("step", "idle")
        msg = "Done."
        if act == "start":
            self.wip = {"step": "full"}
            msg = "Calibration started."
        elif act == "cancel":
            self.wip = {"step": "idle"}
            msg = "Calibration cancelled. Previous calibration (if any) is unchanged."
        elif act == "record_full" and step == "full":
            v, m = self._capture()
            if v is None:
                return {"ok": False, "msg": m}
            w["full"] = v
            w["step"] = "close"
            msg = "Full weight recorded."
        elif act == "valve_closed" and step == "close":
            w["step"] = "drain"
            msg = "Now drain the tank."
        elif act == "record_empty" and step == "drain":
            v, m = self._capture()
            if v is None:
                return {"ok": False, "msg": m}
            if abs(w["full"] - v) < MIN_CAL_COUNTS:
                return {"ok": False, "msg": "Full and empty readings are almost identical (%d counts apart). Is the tank really drained?" % abs(w["full"] - v)}
            w["empty"] = v
            w["step"] = "grams"
            msg = "Empty weight recorded."
        elif act == "grams" and step == "grams":
            try:
                grams = float(q.get("g", ""))
            except ValueError:
                return {"ok": False, "msg": "Enter the grams of water you collected."}
            if grams < 500 or grams > 20000:
                return {"ok": False, "msg": "That doesn't look right for a 2.5 gal tank (expected roughly 7000-9500 g)."}
            scale = grams / (w["full"] - w["empty"])
            old_curve = self.cal.get("curve") if self.cal else None
            self.cal = {"empty": w["empty"], "scale": scale, "cap": grams, "curve": old_curve, "at": epoch()}
            save_json(CALIB_FILE, self.cal)
            self.reset_runtime()
            w["step"] = "open"
            msg = "Scale calibrated. Now open the valve."
        elif act == "start_refill" and step == "open":
            with open(LOG_FILE, "w") as f:
                f.write("epoch,fraction,ml,rate_ml_s,window_s\n")
            self.log_n = 0
            self.last_log_t = mono() - LOG_PERIOD_MS + 5000
            w["step"] = "refill"
            msg = "Logging the refill. Avoid pouring until it finishes."
        elif act == "finish" and step == "refill":
            ok, m = self._finish_refill(q.get("force") == "1")
            if not ok:
                return {"ok": False, "msg": m}
            return {"ok": True, "msg": m}
        elif act == "skip" and step in ("open", "refill"):
            self.wip = {"step": "idle"}
            msg = "Skipped. Pour volumes aren't corrected for refill until a curve is logged."
        else:
            return {"ok": False, "msg": "That action doesn't fit the current step."}
        self._save_wip()
        return {"ok": True, "msg": msg}

    # ---------- dispenser ----------
    # The valve is normally open: at idle (coil off) the sink works like any sink.
    # Flow: start    the user picks an amount; the valve stays OPEN and the screen says "start pouring"
    #       armed    waiting for the scale to see the tap open (same draw detector the manual pours use)
    #       run      the scale meters the pour; the valve closes by itself at the target
    #       settle   valve closed, a couple more seconds of measuring
    #       report   the valve stays CLOSED while the report is up, so the container can't over-fill
    #       done     the user has moved the tap over the sink and shut it; closing the report reopens the
    #                valve and normal water is back.
    def _disp_end(self):
        """Leave dispense mode: de-energize the solenoid so the sink works normally again."""
        valve_set(True)
        self.disp = None
        self.prev_t = None
        self.out_ema = 0.0
        self.draw_cnt = 0
        self.last_draw_t = mono()

    def _disp_close(self, t, why):
        valve_set(False)
        d = self.disp
        if d is None or d["st"] != "run":
            return
        d["st"] = "settle"
        d["why"] = why
        d["tc"] = t
        d["rate_close"] = d["rate"]
        d["settled"] = False

    def emergency(self):
        # Only stops a dispense that is running. At idle the valve must stay open, or a sensor
        # glitch would cut off the sink with no way to restore it.
        try:
            if self.disp is not None and self.disp["st"] == "run":
                self._disp_close(mono(), "error")
        except Exception as e:
            print("emergency:", e)

    def _live(self, d, t):
        """The measured drain rate, once the flow has had time to ramp up and settle."""
        if t - d["t0"] > DISP_LEARN_MS and d["rate"] > 3.0:
            return d["rate"]
        return None

    def _disp_eta(self, d, g, live):
        """Seconds until the valve will be told to close. g = tank level now (ml), live = measured drain
        rate or None. Walks down the learned drain curve (rate falls as the tank empties), scaled to
        how hard the tap is actually open, and allows for the tank refilling while it pours."""
        cap = self.cal["cap"]
        lag = self.settings["lag"]
        cur = self.out_curve
        if not cur:
            if live is None:
                return None
            return max(0.0, (d["target"] - d["ml"] - live * lag) / live)
        f_now = clamp(g / cap, 0.0, 1.0)
        r_now = curve_at(cur, f_now)
        scale = 1.0
        if live is not None and r_now > 1.0:
            scale = clamp(live / r_now, 0.4, 2.5)
        remain = d["target"] - d["ml"] - r_now * scale * lag
        if remain <= 0:
            return 0.0
        steps = 12
        dml = remain / steps
        inflow = self.inflow_est(f_now)
        lvl = g
        tot = 0.0
        for _ in range(steps):
            r = max(curve_at(cur, clamp((lvl - dml * 0.5) / cap, 0.0, 1.0)) * scale, 2.0)
            dt = dml / r
            tot += dt
            lvl = max(0.0, lvl - dml + inflow * dt)
        return tot

    def _disp_sample(self, t, g):
        d = self.disp
        st = d["st"]
        if st == "report":
            return
        cap = self.cal["cap"]
        if st == "armed":
            self._pour_logic(t, g)           # the same draw detector the manual pours use
            p = self.pour
            if p is None:
                return
            # the tap just opened: take the pour over from the detector
            self.pour = None
            self.draw_cnt = 0
            self.full_latch = False
            d.update({"st": "run", "w0": p["w0"], "f0": p["f0"], "f_end": p["f0"], "in_int": p["in_int"],
                      "ml": 0.0, "rate": 0.0, "prev": t, "t0": p["t0"], "pt": t, "pml": 0.0,
                      "settled": False, "why": ""})
            d.pop("p50", None)
            d["eta0"] = self._disp_eta(d, p["w0"], None)     # the initial estimate, before any live flow data
            st = "run"
        dt = (t - d["prev"]) / 1000.0
        d["prev"] = t
        if dt <= 0 or dt > 1.0:
            dt = 0.1
        in_est = self.inflow_est(clamp(g / cap, 0.0, 1.2))
        d["in_int"] += in_est * dt
        vals = self.gbuf.v[-3:]
        ws = sum(vals) / len(vals)
        ml = max(0.0, d["w0"] - ws + d["in_int"])
        s1 = self.gbuf.slope(t, 1000, None, 5)
        if s1 is not None:
            self.slope2 = s1
            a = dt / 0.5 if dt < 0.5 else 1.0
            d["rate"] += a * (max(0.0, in_est - s1) - d["rate"])
        d["ml"] = ml
        d["f_end"] = ws / cap
        self.last_draw_t = t        # keeps the inflow estimator and pour detector out of the way
        self.out_ema = d["rate"]
        if st == "run":
            el = t - d["t0"]
            if ml - d["pml"] >= DISP_STALL_ML:
                d["pml"] = ml
                d["pt"] = t
            if "p50" not in d and ml >= 0.5 * d["target"]:
                e = self._disp_eta(d, ws, self._live(d, t))      # the estimate at the halfway mark
                d["p50"] = [round(el / 1000.0, 2), None if e is None else round(e, 2)]
            if el > DISP_LEARN_MS and d["rate"] > 3.0:           # time-weighted drain rate per fullness bin
                b = int(clamp(ws / cap * OUT_BINS, 0, OUT_BINS - 1))
                d["bs"][b] += d["rate"] * dt
                d["bt"][b] += dt
            if ml + d["rate"] * self.settings["lag"] >= d["target"]:
                self._disp_close(t, "done")
            elif ws < 20:
                self._disp_close(t, "tank empty")
            elif t - d["pt"] > DISP_STALL_MS:
                self._disp_close(t, "no flow")
            elif el > DISP_MAX_MS:
                self._disp_close(t, "timeout")
            elif el > 3000 and ml < DISP_FALSE_ML:
                d["st"] = "armed"            # false trigger (a bump on the tank): go back to waiting
                d["tq"] = t
                d["ml"] = 0.0
                d["rate"] = 0.0
                self.out_ema = 0.0
                self.prev_t = None
                self.draw_cnt = 0
        elif t - d["tc"] >= DISP_SETTLE_MS:
            self._disp_settle(True)

    def _disp_settle(self, learn):
        d = self.disp
        d["settled"] = True
        d["st"] = "report"
        ml = d["ml"]
<<<<<<< Updated upstream
        if learn and self.settings.get("learn", True) and d["why"] == "done" and d["rate_close"] > 5.0:
            lag = self.settings["lag"] + 0.6 * (ml - d["target"]) / d["rate_close"]
            self.settings["lag"] = round(clamp(lag, 0.0, VALVE_LAG_MAX), 3)
=======
        lag0 = self.settings["lag"]
        if learn and d["why"] == "done" and d["rate_close"] > 5.0:
            lag = lag0 + 0.6 * (ml - d["target"]) / d["rate_close"]
            self.settings["lag"] = round(clamp(lag, 0.0, 1.5), 3)
>>>>>>> Stashed changes
            try:
                self._save_settings()
            except Exception as e:
                print("settings save:", e)
        if learn and d["why"] == "done":
            self._learn_out(d)
        dur = (d["tc"] - d["t0"]) / 1000.0
        if ml >= MIN_POUR_ML:
            self._log_pour([epoch(), int(round(ml)), round(dur, 1),
                            round(d["f0"] * 100, 1), round(d["f_end"] * 100, 1)])
<<<<<<< Updated upstream
        if d.get("auto"):
            self._disp_end()        # API pour started with auto=1: nobody to confirm "tap is closed"

    def _disp_begin_run(self, d, t):
        """Baseline the scale and open the valve: the metered part of a dispense starts here."""
        vals = self.gbuf.v[-3:]
        w0 = sum(vals) / len(vals)
        d.update({"w0": w0, "f0": w0 / self.cal["cap"], "f_end": w0 / self.cal["cap"],
                  "in_int": 0.0, "ml": 0.0, "rate": 0.0, "prev": t, "t0": t,
                  "pt": t, "pml": 0.0, "settled": False, "why": ""})
        d["st"] = "run"
        self.in_meas = None
        valve_set(True)
=======
        rep = {"kind": "auto", "why": d["why"], "target": d["target"], "ml": int(round(ml)),
               "dur": round(dur, 1), "avg": round(ml / (dur + lag0), 1) if dur > 0.5 else None,
               "f0": round(d["f0"] * 100, 1), "f_end": round(d["f_end"] * 100, 1)}
        if d["why"] == "done":
            e0 = d.get("eta0")
            p50 = d.get("p50")
            rep["e0"] = None if e0 is None else round(e0, 1)
            rep["e50"] = None if (p50 is None or p50[1] is None) else round(p50[0] + p50[1], 1)
        d["rep"] = self._report(rep)

    def _learn_out(self, d):
        """Fold this pour's drain rate (per fullness bin) into the learned curve and save it."""
        changed = False
        for b in range(OUT_BINS):
            tt = d["bt"][b]
            if tt < 0.4:
                continue
            m = d["bs"][b] / tt
            n = self.out_n[b]
            if n >= 3 and (m < 0.5 * self.out_m[b] or m > 2.0 * self.out_m[b]):
                continue                     # tap barely open (or something odd): don't poison the curve
            k = n + 1 if n < 12 else 12      # running mean at first, then a ~12-pour memory
            self.out_m[b] = m if n == 0 else self.out_m[b] + (m - self.out_m[b]) / k
            self.out_n[b] = k
            changed = True
        if changed:
            self._out_rebuild()
            try:
                save_json(DRAIN_FILE, {"m": [round(x, 2) for x in self.out_m], "n": self.out_n})
            except Exception as e:
                print("drain save:", e)

    def _out_rebuild(self):
        known = [i for i in range(OUT_BINS) if self.out_n[i] > 0]
        if not known:
            self.out_curve = None
            return
        c = [0.0] * OUT_BINS
        for i in range(OUT_BINS):
            if self.out_n[i] > 0:
                c[i] = self.out_m[i]
                continue
            a = None
            b = None
            for k in known:
                if k < i:
                    a = k
                elif k > i and b is None:
                    b = k
            if a is None:
                c[i] = self.out_m[b]
            elif b is None:
                c[i] = self.out_m[a]
            else:
                c[i] = self.out_m[a] + (self.out_m[b] - self.out_m[a]) * (i - a) / (b - a)
        self.out_curve = c
>>>>>>> Stashed changes

    def disp_action(self, act, q):
        d = self.disp
        if act == "start":
            # mode=touchscreen (default): the touchscreen asks for "Open the tap" and the person confirms.
            # mode=force: skip that prompt and start metering right now (the tap must already be open).
            #   auto=1 (force only) also skips "Close the tap" and restores normal water once settled.
            mode = q.get("mode", "touchscreen") or "touchscreen"
            if mode not in ("touchscreen", "force"):
                return {"ok": False, "msg": "mode must be touchscreen or force."}
            force = mode == "force"
            if d is not None and not (force and d["st"] == "confirm1"):
                return {"ok": False, "msg": "A dispense is already in progress."}
            if not self.cal or self.g is None or not self.sensor_ok:
                return {"ok": False, "msg": "Scale not ready."}
            if self.pour is not None:
                return {"ok": False, "msg": "A pour is in progress."}
            try:
                ml = int(float(q.get("ml", "")))
            except ValueError:
                return {"ok": False, "msg": "Bad amount."}
            if ml < 1 or ml > DISP_MAX_ML:
                return {"ok": False, "msg": "Amount must be 1-%d ml." % DISP_MAX_ML}
            if ml > self.g + 20:
                return {"ok": False, "msg": "Only %d ml in the tank." % max(0, int(self.g))}
<<<<<<< Updated upstream
            if force:
                if len(self.gbuf.v) < 3:
                    return {"ok": False, "msg": "Scale not ready."}
                # (replaces a touchscreen request that was still waiting for its confirmation)
                self.disp = nd = {"st": "confirm1", "target": ml, "tq": mono(), "ml": 0.0, "rate": 0.0,
                                  "force": True, "auto": q.get("auto") == "1"}
                self._disp_begin_run(nd, mono())
                return {"ok": True, "msg": "Dispensing.", "mode": "force"}
            valve_set(False)    # close the solenoid: nothing flows, even once the tap is opened
            self.disp = {"st": "confirm1", "target": ml, "tq": mono(), "ml": 0.0, "rate": 0.0}
            return {"ok": True, "msg": "Open the tap.", "mode": "touchscreen"}
=======
            valve_set(True)     # water stays on: the user opens the tap and the scale takes it from there
            self.prev_t = None
            self.draw_cnt = 0
            self.disp = {"st": "armed", "target": ml, "tq": mono(), "ml": 0.0, "rate": 0.0,
                         "bs": [0.0] * OUT_BINS, "bt": [0.0] * OUT_BINS, "settled": False, "why": ""}
            return {"ok": True, "msg": "Start pouring."}
>>>>>>> Stashed changes
        if d is None:
            return {"ok": False, "msg": "No dispense in progress."}
        st = d["st"]
        if act == "cancel" and st == "armed":
            self._disp_end()
<<<<<<< Updated upstream
            return {"ok": True, "msg": "Cancelled. Water is back on, so close the tap if you opened it."}
        if act == "confirm1" and st == "confirm1":
            t = mono()
            if self.g is None or not self.sensor_ok or len(self.gbuf.v) < 3:
                self._disp_end()
                return {"ok": False, "msg": "Scale not ready."}
            self._disp_begin_run(d, t)
            return {"ok": True, "msg": "Dispensing."}
=======
            return {"ok": True, "msg": "Cancelled."}
>>>>>>> Stashed changes
        if act == "stop" and st == "run":
            self._disp_close(mono(), "stopped")
            return {"ok": True, "msg": "Stopped."}
        if act == "done" and st == "report":
            self._disp_end()        # tap is over the sink and shut: valve back to open, water restored
            return {"ok": True, "msg": "Water is back on."}
        return {"ok": False, "msg": "That action doesn't fit the current step."}

    def set_settings(self, q):
        if self.disp is not None:
            return {"ok": False, "msg": "Not while dispensing."}
        try:
            if q.get("full") == "1":
                if not self.cal or self.g is None or not self.sensor_ok:
                    return {"ok": False, "msg": "Scale not ready."}
                if self.pour is not None:
                    return {"ok": False, "msg": "A pour is in progress."}
                vals = self.gbuf.v[-10:]
                g = sum(vals) / len(vals)
                f = g / self.cal["cap"]
                if f < 0.5 or f > SNAP_MAX_FRAC:
                    return {"ok": False, "msg": "The reading is %d%%, too far from full to be drift. Recalibrate instead." % int(f * 100)}
                self._snap_full(mono(), g, "manual")
                return {"ok": True, "msg": "Tank set to 100%."}
            if q.get("drain_reset") == "1":
                self.out_m = [0.0] * OUT_BINS
                self.out_n = [0] * OUT_BINS
                self._out_rebuild()
                remove(DRAIN_FILE)
                return {"ok": True, "msg": "Drain curve cleared."}
            if "cap" in q:
                if not self.cal:
                    return {"ok": False, "msg": "Calibrate first."}
                v = float(q["cap"])
                if v < 1000 or v > 20000:
                    return {"ok": False, "msg": "Total volume must be 1000-20000 ml."}
                self.cal["cap"] = v
                save_json(CALIB_FILE, self.cal)
            if "lag" in q:
                v = float(q["lag"])
                if v < 0 or v > VALVE_LAG_MAX:
                    return {"ok": False, "msg": "Close lead must be 0-%.1f s." % VALVE_LAG_MAX}
                self.settings["lag"] = round(v, 3)
            if "learn" in q:
                self.settings["learn"] = q["learn"] == "1"
            for i in range(8):
                k = "p%d" % i
                if k in q:
                    v = int(float(q[k]))
                    if v < 0 or v > DISP_MAX_ML:
                        return {"ok": False, "msg": "Preset must be 0-%d ml." % DISP_MAX_ML}
                    self.settings["presets"][i] = v
            self._save_settings()
        except ValueError:
            return {"ok": False, "msg": "Bad value."}
        return {"ok": True, "msg": "Saved."}

    # ---------- telemetry for outside programs (GET /api/telemetry) ----------
    def telemetry(self):
        t = mono()
        cal = self.cal
        p = self.pour
        dp = self.disp
        lag = self.settings["lag"]
        d = {"epoch": epoch(), "uptime_s": t // 1000, "ip": self.ip,
             "sensor_ok": self.sensor_ok, "saturated": self.sat, "calibrated": cal is not None,
             "raw": self.med,
             "valve": "open" if valve.value() == VALVE_OPEN_LEVEL else "closed",
             "valve_lead_s": lag, "valve_lead_autotune": self.settings["learn"],
             "busy": p is not None or dp is not None,
             "ready": bool(cal) and self.g is not None and self.sensor_ok and p is None and dp is None}
        if cal and self.g is not None:
            cap = cal["cap"]
            ml = max(0.0, self.g)
            in_rate, src = self.in_display(ml / cap)
            if p is not None or dp is not None:
                out = self.out_ema
                net = self.slope2
            elif self.in_meas is not None:
                out = max(0.0, -self.in_meas)
                net = self.in_meas
            else:
                out = 0.0
                net = self.slope2
            d.update({"state": self.state if self.sensor_ok else "fault",
                      "grams": round(self.g, 1), "ml": int(round(ml)), "capacity_ml": int(round(cap)),
                      "percent": round(ml / cap * 100, 1),
                      "inflow_ml_s": round(in_rate, 2), "outflow_ml_s": round(out, 2),
                      "net_ml_s": round(net, 2),
                      "refill_eta_s": None if self.eta_s is None else int(self.eta_s)})
        else:
            d["state"] = ("idle" if cal else "uncal") if self.sensor_ok else "fault"
        d["pour"] = None
        if p is not None:
            d["pour"] = {"ml": int(round(p["snap"])), "elapsed_s": round((t - p["t0"]) / 1000.0, 1),
                         "peak_ml_s": round(p["peak"], 1)}
        d["dispense"] = None
        if dp is not None:
            d["dispense"] = {"state": dp["st"], "target_ml": dp["target"], "ml": int(round(dp["ml"])),
                             "rate_ml_s": round(dp["rate"], 2),
                             "lead_ml": round(dp["rate"] * lag, 1),   # how early the valve closes at this flow
                             "forced": bool(dp.get("force")), "why": dp.get("why", ""),
                             "settled": dp.get("settled", False)}
        lp = self.last_pour
        d["last_pour"] = None if lp is None else {
            "epoch": lp[0], "ml": lp[1], "duration_s": lp[2], "start_pct": lp[3], "end_pct": lp[4]}
        return d

    # ---------- state for the dashboard ----------
    def snapshot(self):
        t = mono()
        cal = self.cal
        step = self.wip.get("step", "idle")
        d = {"ep": epoch(), "up": t // 1000, "ok": self.sensor_ok, "sat": self.sat,
             "cal": cal is not None, "step": step, "ip": self.ip, "raw": self.med,
             "presets": self.settings["presets"], "maxml": DISP_MAX_ML,
<<<<<<< Updated upstream
             "lag": self.settings["lag"], "learn": self.settings["learn"]}
=======
             "ev": self.ev, "rep": self.last_rep, "drain": sum(1 for n in self.out_n if n)}
>>>>>>> Stashed changes
        dp = self.disp
        if dp is not None:
            st = dp["st"]
            dd = {"st": st, "target": dp["target"], "ml": int(round(dp["ml"])), "why": dp.get("why", "")}
            if st == "run":
                dd["rate"] = round(dp["rate"], 1)
                dd["el"] = round((t - dp["t0"]) / 1000.0, 1)
                e = self._disp_eta(dp, self.g if self.g is not None else dp["w0"], self._live(dp, t))
                dd["eta"] = None if e is None else round(e, 1)
            elif st == "report":
                dd["rep"] = dp.get("rep")
            d["disp"] = dd
        if step in ("full", "drain") or not cal:
            d["stab"] = self.stability()
        if step in ("close", "drain", "grams", "open", "refill"):
            d["wip"] = {"full": self.wip.get("full"), "empty": self.wip.get("empty"), "log_n": self.log_n}
        if cal and self.g is not None:
            cap = cal["cap"]
            ml = max(0.0, self.g)
            f = ml / cap
            in_rate, src = self.in_display(f)
            p = self.pour
            if p is not None or dp is not None:
                out = self.out_ema
                net = self.slope2
            elif self.in_meas is not None:
                out = max(0.0, -self.in_meas)
                net = self.in_meas
            else:
                out = 0.0
                net = self.slope2
            state = self.state if self.sensor_ok else "fault"
            d.update({
                "state": state, "ml": int(round(ml)), "cap": int(round(cap)),
                "pct": round(f * 100, 1), "in": round(in_rate, 2), "in_src": src,
                "win": self.meas_win, "out": round(out, 2), "net": round(net, 2),
                "eta": None if self.eta_s is None else int(self.eta_s), "eta_src": self.eta_src,
                "curve_ok": bool(cal.get("curve")), "corr": round(self.corr, 2),
                "cal_at": cal.get("at", 0),
            })
            if p is not None:
                end = min(t, p["last_draw"] + SETTLE_MS)
                d["pour"] = {"ml": int(round(p["snap"])), "dur": round((end - p["t0"]) / 1000.0, 1),
                             "left": round(max(0.0, (POUR_END_MS - (t - p["last_draw"])) / 1000.0), 1),
                             "peak": round(p["peak"], 1), "f0": round(p["f0"] * 100, 1)}
            d["last"] = self.last_pour
            d["recent"] = self.pours[-10:][::-1]
        elif cal:
            d["state"] = "fault" if not self.sensor_ok else "idle"
            d["cap"] = int(round(cal["cap"]))
        else:
            d["state"] = "uncal" if self.sensor_ok else "fault"
        return d


tank = Tank()
hx = HX711(DT_PIN, SCK_PIN, GAIN_PULSES)


# --------------------------- tasks ---------------------------
async def sensor_task():
    while True:
        try:
            if hx.ready():
                tank.on_sample(hx.read())
                await asyncio.sleep_ms(8)
            else:
                await asyncio.sleep_ms(3)
        except Exception as e:
            print("sensor:", e)
            tank.emergency()
            await asyncio.sleep_ms(100)


async def tick_task():
    wdt = WDT(timeout=8000) if USE_WDT else None
    n = 0
    errs = 0
    while True:
        try:
            tank.tick()
            errs = 0
        except Exception as e:
            print("tick:", e)
            errs += 1
            if errs >= 3:       # a tick that keeps failing freezes the status: start the estimators over
                errs = 0
                try:
                    tank.resync("tick errors")
                except Exception as e2:
                    print("resync:", e2)
        if wdt:
            wdt.feed()
        n += 1
        if n % 5 == 0:
            gc.collect()
        await asyncio.sleep(1)


async def wifi_task():
    global ntp_ok
    wlan = network.WLAN(network.STA_IF)
    try:
        network.hostname(HOSTNAME)
    except Exception:
        pass
    wlan.active(True)
    try:
        wlan.config(pm=0xA11140)  # disable WiFi power save for snappier responses
    except Exception:
        pass
    last_ntp = 0
    while True:
        if not wlan.isconnected():
            tank.ip = ""
            try:
                wlan.connect(WIFI_SSID, WIFI_PASS)
            except Exception as e:
                print("wifi:", e)
            for _ in range(30):
                if wlan.isconnected():
                    break
                await asyncio.sleep(1)
            if wlan.isconnected():
                tank.ip = wlan.ifconfig()[0]
                print("Dashboard: http://%s/" % tank.ip)
        if wlan.isconnected() and ntptime and (not ntp_ok or time.time() - last_ntp > 86400):
            try:
                ntptime.settime()
                ntp_ok = True
                last_ntp = time.time()
            except Exception as e:
                print("ntp:", e)
        await asyncio.sleep(10)


# --------------------------- web server ---------------------------
STATUS = {200: "OK", 404: "Not Found", 405: "Method Not Allowed"}


async def send(w, code, ctype, body):
    if isinstance(body, str):
        body = body.encode()
    w.write(("HTTP/1.0 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
             "Cache-Control: no-store\r\nConnection: close\r\n\r\n" % (code, STATUS.get(code, "OK"), ctype, len(body))).encode())
    w.write(body)
    await w.drain()


async def send_file(w, path, ctype):
    try:
        f = open(path, "rb")
    except OSError:
        await send(w, 404, "text/plain", "not found")
        return
    try:
        size = os.stat(path)[6]
        w.write(("HTTP/1.0 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
                 "Cache-Control: no-store\r\nConnection: close\r\n\r\n" % (ctype, size)).encode())
        while True:
            chunk = f.read(1024)
            if not chunk:
                break
            w.write(chunk)
            await w.drain()
    finally:
        f.close()


async def handle(reader, writer):
    try:
        line = await asyncio.wait_for(reader.readline(), 5)
        if not line:
            return
        while True:
            h = await asyncio.wait_for(reader.readline(), 5)
            if not h or h == b"\r\n":
                break
        parts = line.decode().split(" ")
        method = parts[0]
        target = parts[1] if len(parts) > 1 else "/"
        path, _, qs = target.partition("?")
        if path == "/api/state":
            await send(writer, 200, "application/json", json.dumps(tank.snapshot()))
        elif path == "/api/telemetry":
            await send(writer, 200, "application/json", json.dumps(tank.telemetry()))
        elif path == "/api/stats":
            await send(writer, 200, "application/json", json.dumps(tank.stats_snapshot()))
        elif path == "/api/cal":
            if method != "POST":
                await send(writer, 405, "text/plain", "POST only")
            else:
                q = parse_qs(qs)
                res = tank.cal_action(q.get("action", ""), q)
                await send(writer, 200, "application/json", json.dumps(res))
        elif path == "/api/pour":
            # POST /api/pour?ml=500                    -> touchscreen asks "Open the tap", person confirms
            # POST /api/pour?ml=500&mode=force         -> starts metering immediately
            # POST /api/pour?ml=500&mode=force&auto=1  -> same, and restores normal water when done
            # Stop / finish / cancel a pour with /api/dispense?action=stop|confirm2|cancel
            if method != "POST":
                await send(writer, 405, "text/plain", "POST only")
            else:
                await send(writer, 200, "application/json", json.dumps(tank.disp_action("start", parse_qs(qs))))
        elif path in ("/api/dispense", "/api/settings"):
            if method != "POST":
                await send(writer, 405, "text/plain", "POST only")
            else:
                q = parse_qs(qs)
                if path == "/api/dispense":
                    res = tank.disp_action(q.get("action", ""), q)
                else:
                    res = tank.set_settings(q)
                await send(writer, 200, "application/json", json.dumps(res))
        elif path in ("/", "/index.html"):
            await send_file(writer, "index.html", "text/html; charset=utf-8")
        elif path == "/refill_log.csv":
            await send_file(writer, LOG_FILE, "text/csv")
        elif path == "/pours.csv":
            await send_file(writer, POURS_FILE, "text/csv")
        elif path == "/drain.json":
            await send_file(writer, DRAIN_FILE, "application/json")
        elif path == "/snaps.csv":
            await send_file(writer, SNAP_FILE, "text/csv")
        else:
            await send(writer, 404, "text/plain", "not found")
    except Exception as e:
        print("http:", e)
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


async def main():
    tank.load()
    asyncio.create_task(sensor_task())
    asyncio.create_task(tick_task())
    asyncio.create_task(wifi_task())
    for _ in range(45):
        if tank.ip:
            break
        await asyncio.sleep(1)
    await asyncio.start_server(handle, "0.0.0.0", 80)
    while True:
        await asyncio.sleep(3600)


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()