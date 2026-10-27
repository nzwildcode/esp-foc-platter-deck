# PlatterDeck Windows app

Rust deck app. It receives the platter's angle over Wi-Fi from the ESP32 FOC
board, and decodes, scratches, cues and plays the track on Windows. A
mouse-driven simulator stands in for the platter when no board is connected.

## Prototype observations

| Mode | Audio rate wobble (rms) | Largest block step | Mean rate |
|---|---|---|---|
| STEADY | 0.000 % | 0.000 % | 1.000x |
| VINYL | 0.28 % | 0.5 % | 1.000x |

These are software audio-rate observations recorded in earlier project notes;
raw captures are not included. They are not a standardised acoustic
wow-and-flutter test. The engine uses a configured 40 ms Wi-Fi playback delay;
actual motion-to-sound latency depends on the audio device and buffering.

---

## Run

Join the board's hotspot **PlatterDeck** and start `PlatterDeck.exe`. The
board endpoint is `192.168.4.1:19704`. Set the Wi-Fi adapter to Maximum
Performance where available; power saving can cause large latency spikes.

- **STEADY:** the track runs from the audio clock at the pitch-fader tempo.
- **VINYL:** the platter sets position and speed, including reverse.
- **33 / 45** set the platter speed only; the track plays at 1.00x on either.
- The app starts in FREE. PLAY spins the platter; after E-STOP, press FREE
  MOTOR before PLAY.

| Control | Function |
|---|---|
| Pitch fader | Tempo and motor speed; range 8 / 16 / 50 % |
| − / + (hold) | Bend the pitch 4 % to nudge the beat |
| GAIN | Output level |
| START / BRAKE | Platter wind-up and wind-down time, 0–3 s |
| PLAY / PAUSE, Space | Spin / brake |
| CUE, pads 1–4 | Cue and hot cues |
| Waveform | Overview and seek |
| extra lead | Raise if scratches feel late |

Command line:

| Option | Effect |
|---|---|
| `--sim` | Use the simulator |
| `--vinyl` | Start in VINYL |
| `--list` | List audio outputs |
| `--device NAME` / `--buffer N` | Output device / ring buffer in frames |
| `--debug-log X.csv` | Log every platter sample and audio block |
| `--selftest [--vinyl] --secs N [track]` | Headless, silent run that checks the audio rate against the platter |

---

## How VINYL follows the platter

Every 10 ms audio block:

1. Take the platter angle for the moment the block reaches the DAC, 40 ms
   behind the wall clock, interpolated from 512 ms of timestamped history.
2. Fit two least-squares velocities: 24 ms (fast) and 160 ms (flywheel).
3. Use the flywheel while the platter runs free. When a hand deviates it by
   more than 12 %, switch to the fast velocity at once; blend back 300 ms
   after it settles.
4. Ramp the rate across the block, with a slow correction toward the exact
   platter position so no error accumulates.

Audio goes out through WASAPI shared mode (cpal) on an MMCSS thread with a
20 ms ring buffer.

---

## Tools

| Tool | Purpose |
|---|---|
| `tools/harmonic_compensate.py <dir>` | Identify the board's cogging compensation and save it to NVS; about 25 minutes, on the board's hotspot |
| `tools/platter_log.py X.csv [--live]` | Analyse a `--debug-log` file: link rate, gaps, speed ripple spectrum, audio rate |
| `platter-cli --mode spin --rpm 33.333 --dump X.csv` | Spin the platter from the command line and log telemetry |

The Python tools need `numpy` and `websocket-client`, and
`harmonic_compensate.py` needs the release build of `platter-cli`. Run one
controller at a time.

See the [first-use guide](../docs/getting-started.md) for dependency installation,
FOC calibration and creation of the output directory. Identification replaces
the board's previous compensation and moves the rotor.

---

## Build

```powershell
cargo test --workspace --locked
cargo build --release --workspace --locked # target\release\platterdeck.exe, platter-cli.exe
```

| Crate | Contents |
|---|---|
| `crates/proto` | Payload frames and the UDP envelope |
| `crates/engine` | Wi-Fi link, platter history, deck (STEADY / VINYL), audio output, simulator, debug log |
| `crates/app` | egui interface and self-test |
| `crates/cli` | Diagnostics and bench spins |

---

## Links

- [Firmware](../PlatterDeck_FOC/README.md) · [Project overview](../README.md)
- [cpal](https://github.com/RustAudio/cpal) · [egui](https://github.com/emilk/egui) · [symphonia](https://github.com/pdeljanov/Symphonia)
