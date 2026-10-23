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

Wiring is the same as your test script: DT -> GP14, SCK -> GP15.
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

CAP_MS = 8000              # window used when recording full/empty weights
DRIFT_MAX = 6000           # max counts of drift allowed while recording a weight
MIN_CAL_COUNTS = 20000     # full and empty must differ by at least this many counts
LOG_PERIOD_MS = 30000      # refill log interval during calibration
# ================================================================

CALIB_FILE = "calib.json"
WIP_FILE = "cal_wip.json"
LOG_FILE = "refill_log.csv"
POURS_FILE = "pours.csv"
EPOCH_OFFSET = 946684800   # MicroPython epoch is 2000-01-01

ntp_ok = False


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

    # ---------- persistence ----------
    def load(self):
        self.cal = load_json(CALIB_FILE, None)
        wip = load_json(WIP_FILE, None)
        if wip:
            self.wip = wip
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
        g = (med - self.cal["empty"]) * self.cal["scale"]
        self.g = g
        self.gbuf.add(t, g)
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
        if out_inst > OUT_KEEP:
            p["last_draw"] = t
            self.last_draw_t = t
        idle = t - p["last_draw"]
        if idle <= SETTLE_MS:
            p["snap"] = p["ml"]
            p["f_end"] = ws / cap
            if self.out_ema > p["peak"]:
                p["peak"] = self.out_ema
        if idle > POUR_END_MS:
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
        t0 = gb.t[i]
        self.pour = {
            "t0": t0, "w0": w0, "in_int": in_est * (t - t0) / 1000.0,
            "last_draw": t, "ml": 0.0, "snap": 0.0, "peak": 0.0,
            "f0": w0 / cap, "f_end": w0 / cap,
        }
        self.draw_cnt = 0

    def _end_pour(self, t):
        p = self.pour
        self.pour = None
        self.draw_cnt = 0
        ml = p["snap"]
        dur = (p["last_draw"] - p["t0"]) / 1000.0
        if ml < MIN_POUR_ML:
            return
        rec = [epoch(), int(round(ml)), round(dur, 1), round(p["f0"] * 100, 1), round(p["f_end"] * 100, 1)]
        self.pours.append(rec)
        if len(self.pours) > 20:
            self.pours.pop(0)
        self.last_pour = rec
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
        if not self.cal or not self.sensor_ok or len(self.gbuf.v) < 5:
            return
        vals = [v for tt, v in zip(self.gbuf.t, self.gbuf.v) if tt >= t - 1000]
        if not vals:
            return
        g1 = sum(vals) / len(vals)
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

        # keep the live-supply correction up to date
        cur = self.cal.get("curve")
        if meas is not None and cur and f < 0.97:
            cr = curve_at(cur, f)
            if cr > 0.03:
                target = clamp(meas / cr, 0.0, 5.0)
                self.corr += 0.15 * (target - self.corr)

        in_rate, _ = self.in_display(f)
        if self.pour is not None:
            self.state = "pour"
        elif f >= FULL_FRAC:
            self.state = "full"
        elif in_rate > 0.03:
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
                    r = max(curve_at(cur, f + (k + 0.5) * df) * self.corr, 0.01)
                    tot += df * cap / r
                self.eta_s = tot
                self.eta_src = "curve"
            elif in_rate > 0.03:
                self.eta_s = (FULL_FRAC - f) * cap / in_rate
                self.eta_src = "linear"

        # calibration refill logging
        if self.wip.get("step") == "refill":
            self._refill_logging(t, f, g1, meas, since)

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

    # ---------- state for the dashboard ----------
    def snapshot(self):
        t = mono()
        cal = self.cal
        step = self.wip.get("step", "idle")
        d = {"ep": epoch(), "up": t // 1000, "ok": self.sensor_ok, "sat": self.sat,
             "cal": cal is not None, "step": step, "ip": self.ip, "raw": self.med}
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
            if p is not None:
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
            await asyncio.sleep_ms(100)


async def tick_task():
    wdt = WDT(timeout=8000) if USE_WDT else None
    n = 0
    while True:
        try:
            tank.tick()
        except Exception as e:
            print("tick:", e)
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
        elif path == "/api/cal":
            if method != "POST":
                await send(writer, 405, "text/plain", "POST only")
            else:
                q = parse_qs(qs)
                res = tank.cal_action(q.get("action", ""), q)
                await send(writer, 200, "application/json", json.dumps(res))
        elif path in ("/", "/index.html"):
            await send_file(writer, "index.html", "text/html; charset=utf-8")
        elif path == "/refill_log.csv":
            await send_file(writer, LOG_FILE, "text/csv")
        elif path == "/pours.csv":
            await send_file(writer, POURS_FILE, "text/csv")
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