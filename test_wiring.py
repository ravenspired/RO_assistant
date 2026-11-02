"""
HX711 + 3x parallel load cells test for Raspberry Pi Pico (RP2040), MicroPython.

Wiring (Pico -> HX711):
    3V3(OUT) pin 36 -> VCC
    GND             -> GND
    GP14            -> DT
    GP15            -> SCK
Load cells (joined in parallel) -> E+, E-, A+, A-

Run in Thonny (or any REPL). No calibration here: this only checks that the
chip responds, the signal is sane, noise is reasonable, and each cell reacts.
"""
from machine import Pin, disable_irq, enable_irq
import time

DT_PIN = 14
SCK_PIN = 15


class HX711:
    def __init__(self, dt, sck, gain_pulses=1):
        # gain_pulses: 1 = ch A gain 128, 2 = ch B gain 32, 3 = ch A gain 64
        self.dt = Pin(dt, Pin.IN, Pin.PULL_UP)
        self.sck = Pin(sck, Pin.OUT, value=0)
        self.gain_pulses = gain_pulses

    def ready(self):
        return self.dt.value() == 0

    def read(self, timeout_ms=1000):
        """Return one signed 24-bit reading, or None on timeout."""
        t0 = time.ticks_ms()
        while not self.ready():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                return None
        val = 0
        state = disable_irq()  # SCK high >60 us would power down the chip
        for _ in range(24):
            self.sck.value(1)
            self.sck.value(0)
            val = (val << 1) | self.dt.value()
        for _ in range(self.gain_pulses):
            self.sck.value(1)
            self.sck.value(0)
        enable_irq(state)
        if val & 0x800000:
            val -= 0x1000000
        return val


def mean_std(xs):
    n = len(xs)
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / n
    return m, var ** 0.5


def grab(hx, n):
    out = []
    for _ in range(n):
        v = hx.read()
        if v is None:
            return None
        out.append(v)
    return out


def stream(hx, seconds, baseline, label):
    """Print live readings for `seconds`; return the largest deviation seen."""
    print("\n>>> %s (%d s)" % (label, seconds))
    t_end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    best = 0
    last_print = 0
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        v = hx.read()
        if v is None:
            print("timeout")
            continue
        d = v - baseline
        if abs(d) > abs(best):
            best = d
        if time.ticks_diff(time.ticks_ms(), last_print) > 250:
            last_print = time.ticks_ms()
            print("raw=%9d  delta=%+9d" % (v, d))
    return best


def main():
    hx = HX711(DT_PIN, SCK_PIN)

    # --- 1. Does the chip respond? ---
    print("1) Checking HX711 response...")
    first = hx.read(timeout_ms=2000)
    if first is None:
        print("FAIL: HX711 never became ready.")
        print("  Check VCC/GND, DT->GP%d, SCK->GP%d, and that the board is powered." % (DT_PIN, SCK_PIN))
        return
    print("OK: got reading", first)

    # --- 2. Baseline + sanity ---
    print("\n2) Baseline (leave scale unloaded and still)...")
    time.sleep(1)
    samples = grab(hx, 40)
    if samples is None:
        print("FAIL: timeouts while sampling.")
        return
    base, noise = mean_std(samples)
    print("mean=%.0f  std=%.1f counts  (min=%d max=%d)" % (base, noise, min(samples), max(samples)))

    if max(samples) >= 8388607 or min(samples) <= -8388608:
        print("WARN: ADC is saturated. Likely E+/E-/A+/A- miswired, an open wire, or a bad join.")
    elif base == 0 and noise == 0:
        print("WARN: constant 0. Inputs may be shorted or disconnected.")
    elif noise > 2000:
        print("WARN: very noisy. Check for loose joints, long unshielded wires, or a weak supply.")
    else:
        print("Baseline looks sane.")

    # --- 3. Does each cell respond? ---
    print("\n3) Press down firmly on each cell's corner in turn (finger or ~1 kg is plenty).")
    time.sleep(2)
    threshold = max(10 * noise, 500)
    results = []
    for i in (1, 2, 3):
        # re-baseline between presses so slow drift doesn't confuse results
        b, _ = mean_std(grab(hx, 20) or [base])
        peak = stream(hx, 6, b, "Press on CELL %d now" % i)
        results.append(peak)
        time.sleep(1)

    # --- Summary ---
    print("\n=== SUMMARY ===")
    print("baseline noise: %.1f counts std" % noise)
    all_ok = True
    for i, peak in enumerate(results, 1):
        ok = abs(peak) > threshold
        all_ok = all_ok and ok
        print("cell %d: peak delta %+d -> %s" % (i, peak, "responds" if ok else "NO CLEAR RESPONSE"))
    signs = [p > 0 for p in results if abs(p) > threshold]
    if signs and not (all(signs) or not any(signs)):
        print("WARN: cells moved in opposite directions. One cell's A+/A- (or E+/E-) is likely reversed.")
    elif signs and not any(signs):
        print("Note: pressing gives NEGATIVE counts. Swap A+ and A- (or just flip the sign in code).")
    if all_ok and signs and (all(signs) or not any(signs)):
        print("All three cells respond with consistent polarity. Good to go.")


main()