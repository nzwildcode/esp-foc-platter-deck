"""Identify and compensate the platter's angle-locked speed ripple, harmonic
by harmonic, and store the result on the board.

  python tools/harmonic_compensate.py <scratch_dir> [spin_seconds] [slots]

Takes 20-25 minutes for 40 harmonics; harmonics are saved at the end.

For each significant harmonic k of shaft angle (largest first, three per
spin): inject a small q-voltage sinusoid at k, solve the loop's complex
response R at that frequency, apply the cancelling sinusoid U = -C/R,
measure, and refine U by secant steps on the measured residual. Probes and
steps are kept small: the loop's response near its resonant peak is
strongly amplitude dependent (a 0.2 V probe at 48x measured half the
small-signal gain and the cancellation landed 90 degrees off). The result
is in volts of cogging torque versus shaft angle, so it holds at any speed
and survives a retune of the velocity loop. Finally it is saved to NVS.
Motor spins at 33 1/3 rpm over the CLI; the board sums the harmonics into
its 1024-bin table.
"""
import json
import os
import subprocess
import sys
import time

import numpy as np
import websocket

SP = sys.argv[1]
SECS = int(sys.argv[2]) if len(sys.argv) > 2 else 10
SLOTS = int(sys.argv[3]) if len(sys.argv) > 3 else 48
sys.path.insert(0, os.path.dirname(__file__))
import platter_log  # noqa: E402

CLI = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "target", "release", "platter-cli.exe"))
UNITS = 65536.0
RPM = 33.333
CANDIDATES = tuple(range(1, 121))
GROUP = 3           # harmonics identified per spin (the board's probe slots)
# Harmonics below their threshold are left alone. The deck's flywheel
# averages out everything above ~5 Hz (9x/rev at 33 rpm), so the low
# orders are the audible wow and get chased closer to the 0.06 rpm floor.
def stop_rpm(k):
    return 0.08 if k <= 12 else 0.3


def done_rpm(k):
    return 0.06 if k <= 12 else 0.2


# Typical small-signal response, rpm/V. The velocity loop rejects the low
# orders (below its ~7 Hz crossover), so the prior falls with k there.
def r_prior(k):
    return 150.0 * min(1.0, k / 12.0)


PROBE_MIN, PROBE_MAX = 0.004, 0.06
MAX_ITER = 4


def reconnect():
    subprocess.run(["netsh", "wlan", "connect", "name=PlatterDeck"], capture_output=True)
    time.sleep(5)


def ws_send(actions, want_state=False):
    time.sleep(0.3)
    for attempt in range(3):
        try:
            ws = websocket.create_connection("ws://192.168.4.1/ws", timeout=5)
            break
        except Exception as e:
            print(f"ws connect failed ({e}); reconnecting")
            sys.stdout.flush()
            reconnect()
    else:
        sys.exit("board unreachable")
    end = time.time() + 2
    while time.time() < end:
        try:
            if json.loads(ws.recv()).get("type") == "state":
                break
        except Exception:
            break
    for a in actions:
        ws.send(json.dumps(a))
        time.sleep(0.3)
    st = None
    if want_state:
        ws.send(json.dumps({"action": "getState"}))
        end = time.time() + 3
        while time.time() < end:
            try:
                m = json.loads(ws.recv())
            except Exception:
                break
            if m.get("type") == "state":
                st = m
                break
    ws.close()
    return st


def spin(tag):
    path = f"{SP}/comp_{tag}.csv"
    subprocess.run([CLI, "--mode", "spin", "--rpm", str(RPM), "--torque", "1.0",
                    "--secs", str(SECS), "--dump", path], capture_output=True, text=True)
    return path


def speed_vs_angle(path, skip_s=4.0):
    s, _ = platter_log.parse(path)
    if len(s) < 500:
        return None, None
    a = np.array(s, dtype=np.float64)
    t = a[:, 0] * 1e-6
    # Absolute board angle (column 6): the phase reference must be the
    # physical shaft, not where the platter sat when the link came up.
    ang = np.unwrap(a[:, 5] / UNITS * 2 * np.pi) / (2 * np.pi)
    keep = t >= t[0] + skip_s
    t, ang = t[keep], ang[keep]
    r0, r1 = np.ceil(ang[0]), np.floor(ang[-1])
    if r1 - r0 < 2:
        return None, None
    grid = np.linspace(r0, r1, int((r1 - r0) * 2048) + 1)
    tg = np.interp(grid, ang, t)
    w = np.diff(grid) / np.diff(tg) * 60.0
    # ~1.4 degree boxcar: suppresses encoder quantisation noise in the
    # angle-grid derivative without touching harmonics below ~250x.
    w = np.convolve(w, np.ones(8) / 8, mode="same")
    th = 2 * np.pi * (grid[:-1] + grid[1:]) / 2
    return w, th


def harmonics(w, th):
    return {k: 2.0 * np.mean(w * np.exp(-1j * k * th)) for k in CANDIDATES}


def rms(w):
    return float(np.sqrt(np.mean((w - w.mean()) ** 2)))


def polar(c):
    return f"{abs(c):.3f}@{np.degrees(np.angle(c)):+.0f}"


def measure(tag):
    for attempt in range(3):
        w, th = speed_vs_angle(spin(tag))
        if w is not None:
            return w, th, harmonics(w, th)
        print(f"{tag}: no telemetry, reconnecting")
        sys.stdout.flush()
        reconnect()
    sys.exit("no telemetry")


def report(label, w, h):
    top = sorted(h.items(), key=lambda kv: -abs(kv[1]))[:8]
    print(f"{label}: rms {rms(w):5.2f} rpm | " + " ".join(f"{k}x={abs(c):.2f}" for k, c in top))
    sys.stdout.flush()


def set_comp(slot, k, U):
    # Board applies amp*sin(k*angle + phase); the analysis uses cos, hence pi/2.
    amp = float(min(abs(U), 0.45))
    phase = float(np.angle(U) + np.pi / 2)
    ws_send([{"action": "setHarmonic", "slot": slot, "k": k, "amp": amp, "phase": phase}])
    return amp * np.exp(1j * (phase - np.pi / 2))


def set_probes(items):
    acts = []
    for i in range(GROUP):
        if i < len(items):
            k, amp = items[i]
            acts.append({"action": "probe", "slot": i, "k": k, "amp": float(amp), "phase": 0.0})
        else:
            acts.append({"action": "probe", "slot": i, "k": 0, "amp": 0, "phase": 0})
    ws_send(acts)


def probe_group(ks, h, scale=1.0):
    """One spin with a small probe on every k in ks. Returns k -> (R, moved, amp)."""
    amps = {k: float(np.clip(scale * max(abs(h[k]), 0.8) / r_prior(k), PROBE_MIN, PROBE_MAX)) for k in ks}
    set_probes([(k, amps[k]) for k in ks])
    wp, _, hp = measure("probe_" + "_".join(map(str, ks)))
    set_probes([])
    out = {}
    for k in ks:
        u = amps[k] * np.exp(-1j * np.pi / 2)
        d = hp[k] - h[k]
        out[k] = (d / u, abs(d), amps[k])
    return out


ws_send([{"action": "clearHarmonics"}])
set_probes([])

w, th, h = measure("base")
rms0 = rms(w)
report("baseline", w, h)
done = set()
assigned = {}   # k -> (slot, U, secant gain)
slot = 0
while slot < SLOTS:
    remaining = sorted(((abs(c), k) for k, c in h.items() if k not in done and abs(c) >= stop_rpm(k)),
                       reverse=True)
    if not remaining:
        break
    ks = [k for _, k in remaining[:min(GROUP, SLOTS - slot)]]
    resp = probe_group(ks, h)
    weak = [k for k in ks if resp[k][1] < 0.25 and resp[k][2] < PROBE_MAX]
    if weak:
        resp.update(probe_group(weak, h, scale=3.0))
    active = {}
    for k in ks:
        R, moved, amp = resp[k]
        print(f"k={k}: baseline {abs(h[k]):.2f} rpm; probe {amp:.3f} V moved it {moved:.2f} rpm "
              f"-> R {abs(R):.0f} rpm/V @ {np.degrees(np.angle(R)):+.0f} deg")
        if moved < 0.15 or abs(R) < 5:
            print(f"k={k}: no usable response, skipped")
            done.add(k)
            continue
        active[k] = {"slot": slot, "U": -h[k] / R, "points": [(0j, h[k])],
                     "best": (abs(h[k]), 0j, R), "Rs": R, "stop": False}
        slot += 1
        done.add(k)
    sys.stdout.flush()
    if not active:
        continue
    last = None
    for it in range(MAX_ITER):
        for k, a in active.items():
            if not a["stop"]:
                a["U"] = set_comp(a["slot"], k, a["U"])
        wa, tha, ha = measure("apply_" + "_".join(map(str, ks)) + f"_it{it}")
        last = (wa, tha, ha)
        line = []
        for k, a in active.items():
            if a["stop"]:
                continue
            ca = ha[k]
            Up, cp = a["points"][-1]
            Rs = (ca - cp) / (a["U"] - Up) if a["U"] != Up else a["Rs"]
            a["points"].append((a["U"], ca))
            if abs(ca) < a["best"][0]:
                a["best"] = (abs(ca), a["U"], Rs)
            line.append(f"{k}x {polar(a['U'])} V -> {abs(ca):.2f}")
            if abs(ca) < done_rpm(k) or abs(Rs) < 5:
                a["stop"] = True
                continue
            a["Rs"] = Rs
            a["U"] = a["U"] - ca / Rs
        print(f"   it{it}: " + "; ".join(line) + f"  (rms {rms(wa):.2f})")
        sys.stdout.flush()
        if all(a["stop"] for a in active.values()):
            break
    reverted = False
    for k, a in active.items():
        if a["points"][-1][0] != a["best"][1]:
            set_comp(a["slot"], k, a["best"][1])
            reverted = True
        assigned[k] = (a["slot"], a["best"][1], a["best"][2])
    if reverted:
        w, th, h = measure("after_" + "_".join(map(str, ks)))
    else:
        w, th, h = last
    report("after " + "/".join(f"{k}x" for k in active), w, h)

# Polish: one secant step on anything that drifted while the rest was being
# identified (winding temperature moves the volts-per-torque scale).
drift = [k for k in assigned if abs(h[k]) > 1.5 * done_rpm(k) and abs(assigned[k][2]) >= 5]
if drift:
    old = {k: assigned[k] for k in drift}
    for k in drift:
        sl, U, Rs = old[k]
        assigned[k] = (sl, set_comp(sl, k, U - h[k] / Rs), Rs)
    w2, th2, h2 = measure("polish")
    bad = [k for k in drift if abs(h2[k]) >= abs(h[k])]
    print("polish: " + " ".join(f"{k}x {abs(h[k]):.2f}->{abs(h2[k]):.2f}" for k in drift))
    sys.stdout.flush()
    for k in bad:
        assigned[k] = old[k]
        set_comp(old[k][0], k, old[k][1])
    if bad:
        w, th, h = measure("polish2")
    else:
        w, th, h = w2, th2, h2

report("final", w, h)
print(f"total ripple rms {rms0:.2f} -> {rms(w):.2f} rpm; speed min {w.min():.2f} max {w.max():.2f}; "
      f"{len(assigned)} harmonics")
st = ws_send([{"action": "saveConfig"}], want_state=True)
if st:
    print(f"saved {len(st.get('harmonics', []))} harmonics; table peak {st.get('compPeakV')} V; "
          f"clips {st.get('ffClips')}")
    print("harmonics:", st.get("harmonics"))
