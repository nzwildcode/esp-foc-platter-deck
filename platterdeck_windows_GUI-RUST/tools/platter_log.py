"""Analyse a PlatterDeck debug tap (platterdeck --debug-log X / platter-cli --dump X).

  python tools/platter_log.py LOG            whole file
  python tools/platter_log.py LOG --last 5   last 5 s
  python tools/platter_log.py LOG --live     rolling 2 s summary every second

Per window it reports what the encoder saw (rpm from angle differences,
its ripple spectrum, glitches, link gaps) and what the audio engine did
with it (rate, position error, flags). This separates link problems
(gaps, delay), encoder/SPI problems (glitches, rest noise) and motor
control problems (ripple at a mechanical or electrical frequency).
"""
import argparse
import math
import sys
import time

import numpy as np

UNITS = 65536.0


def parse(path):
    samples, blocks = [], []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            p = line.rstrip("\n").split(",")
            try:
                if p[0] == "S" and len(p) >= 6:
                    samples.append((int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5]), int(p[6]) if len(p) > 6 else 0))
                elif p[0] == "B" and len(p) >= 8:
                    blocks.append((int(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5]), int(p[6]), p[7]))
            except ValueError:
                continue
    return samples, blocks


def peaks(freqs, amp, n=3):
    out = []
    order = np.argsort(amp)[::-1]
    for i in order:
        f = freqs[i]
        if f < 0.3:
            continue
        if all(abs(f - g) > 1.5 for g, _ in out):
            out.append((f, amp[i]))
        if len(out) == n:
            break
    return out


def summarise(samples, blocks, label):
    lines = [f"== {label} =="]
    if len(samples) < 20:
        lines.append(f"samples: {len(samples)} (not enough)")
        return "\n".join(lines)
    s = np.array(samples, dtype=np.float64)
    t = s[:, 0] * 1e-6
    a = s[:, 1] / UNITS
    vel_board = s[:, 2] * 1e-3 * 60 / (2 * math.pi)
    status = s[:, 3].astype(int)
    delay = s[:, 4] * 1e-3
    span = t[-1] - t[0]
    dt = np.diff(t)
    da = np.diff(a)
    lines.append(
        f"samples {len(s)} over {span:.2f} s = {len(s) / max(span, 1e-9):.0f}/s; "
        f"gaps>10ms {int((dt > 0.010).sum())} max gap {dt.max() * 1e3:.1f} ms; "
        f"link delay mean {delay.mean():.1f} max {delay.max():.1f} ms"
    )
    modes = {0: "FREE", 1: "SPIN", 2: "BRAKE", 3: "ESTOP"}
    mode_counts = {}
    for st in status:
        mode_counts[modes[(st >> 4) & 3]] = mode_counts.get(modes[(st >> 4) & 3], 0) + 1
    flags = []
    if (status & 1).any():
        flags.append("ENABLED")
    if (status & 4).any():
        flags.append("FAULT")
    if (status & 8).any():
        flags.append("TIMEOUT")
    lines.append(f"board mode {mode_counts}; flags seen {flags or 'none'}")
    # Glitches: an angle step no platter can make in one sample period.
    per_s = da / np.maximum(dt, 1e-6)
    glitch = np.abs(per_s) > 20.0  # 20 rev/s = 1200 rpm
    lines.append(f"glitches (>20 rev/s between samples): {int(glitch.sum())}")
    # rpm over 20 ms windows from angle, the truth the audio must follow.
    w = 10
    if len(a) > w + 2:
        rpm = (a[w:] - a[:-w]) / (t[w:] - t[:-w]) * 60.0
        lines.append(
            f"rpm from angle (20 ms windows): mean {rpm.mean():.2f} sd {rpm.std():.2f} "
            f"min {rpm.min():.2f} max {rpm.max():.2f} p-p {rpm.max() - rpm.min():.2f}"
        )
        lines.append(
            f"rpm reported by board: mean {vel_board.mean():.2f} sd {vel_board.std():.2f} "
            f"min {vel_board.min():.2f} max {vel_board.max():.2f}"
        )
        # Rest noise: angle p-p in the window when the platter is nearly still.
        if abs(rpm.mean()) < 0.5:
            lines.append(f"angle p-p at rest: {(a.max() - a.min()) * UNITS:.0f} units = {(a.max() - a.min()) * 16384:.1f} encoder counts")
        # Ripple spectrum of the per-sample velocity, resampled to 500 Hz.
        if span > 0.5 and abs(rpm.mean()) > 2:
            grid = np.arange(t[0], t[-1], 0.002)
            ag = np.interp(grid, t, a)
            v = np.diff(ag) / 0.002 * 60.0  # rpm per sample
            v = v - v.mean()
            win = np.hanning(len(v))
            spec = np.abs(np.fft.rfft(v * win)) * 2.0 / win.sum()
            freqs = np.fft.rfftfreq(len(v), 0.002)
            top = peaks(freqs, spec)
            rev_hz = abs(rpm.mean()) / 60.0
            notes = []
            for f, amp in top:
                tag = ""
                if abs(f - rev_hz) < 0.15 * rev_hz + 0.1:
                    tag = " (once per rev: eccentric magnet/encoder offset)"
                elif abs(f - 7 * rev_hz) < 0.15 * 7 * rev_hz:
                    tag = " (7x: electrical, per pole pair; alignment/zero angle)"
                elif abs(f - 84 * rev_hz) < 0.15 * 84 * rev_hz:
                    tag = " (84x: cogging, 12N14P)"
                elif abs(f - 12 * rev_hz) < 0.15 * 12 * rev_hz:
                    tag = " (12x: slot count)"
                elif abs(f - 14 * rev_hz) < 0.15 * 14 * rev_hz:
                    tag = " (14x: pole count)"
                notes.append(f"{f:.1f} Hz {amp:.2f} rpm{tag}")
            lines.append("ripple peaks: " + "; ".join(notes) + f"  [1/rev = {rev_hz:.2f} Hz]")
    if blocks:
        b = np.array([(x[0], x[1], x[2], x[3], x[4], x[5]) for x in blocks], dtype=np.float64)
        fl = [x[6] for x in blocks]
        rate = b[:, 2]
        moving = rate[np.abs(rate) > 0.01]
        lines.append(
            f"audio blocks {len(b)}: rate mean {rate.mean():.4f} sd {rate.std():.4f} "
            f"(moving: sd {moving.std() if len(moving) else 0:.4f}) pos_err sd {b[:, 4].std():.1f} samples; "
            f"lead {b[:, 5].mean() / 1000:.1f} ms; "
            f"ready {sum('r' in f for f in fl)} stale {sum('s' in f for f in fl)} "
            f"stationary {sum('t' in f for f in fl)} steady {sum('y' in f for f in fl)} "
            f"hands-on {sum('h' in f for f in fl)}"
        )
        if len(moving) > 20:
            d = np.diff(moving)
            lines.append(f"block-to-block rate steps: max {np.abs(d).max():.4f} (a 100 Hz stutter shows here)")
    return "\n".join(lines)


def window(samples, blocks, last_s):
    if not samples:
        return samples, blocks
    t_end = samples[-1][0]
    t0 = t_end - int(last_s * 1e6)
    return [x for x in samples if x[0] >= t0], [x for x in blocks if x[0] >= t0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--last", type=float, default=None, help="only the last N seconds")
    ap.add_argument("--live", action="store_true", help="rolling summary every second")
    ap.add_argument("--window", type=float, default=2.0, help="live window, seconds")
    ap.add_argument("--seconds", type=float, default=30.0, help="live duration")
    args = ap.parse_args()
    if args.live:
        end = time.time() + args.seconds
        while time.time() < end:
            samples, blocks = parse(args.log)
            samples, blocks = window(samples, blocks, args.window)
            print(summarise(samples, blocks, time.strftime("%H:%M:%S")))
            sys.stdout.flush()
            time.sleep(1.0)
        return
    samples, blocks = parse(args.log)
    if args.last:
        samples, blocks = window(samples, blocks, args.last)
    print(summarise(samples, blocks, args.log))


if __name__ == "__main__":
    main()
