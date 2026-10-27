# PlatterDeck firmware

ESP32 firmware for the FOC_ESP32 board (ESP32-WROOM-32E, 303FOCESP01 /
LCT-279) with a GM2804 motor and an AS5048A encoder. It runs a SimpleFOC
velocity loop with cogging compensation, streams 500 Hz timestamped platter
angles over Wi-Fi and serves a browser dashboard.

## Prototype observations

Earlier project notes report 0.61 rpm RMS speed ripple in 20 ms windows at
33 1/3 rpm (0.60 at 45 rpm) after compensation. Raw captures are not included;
these are prototype observations, not a guarantee for another assembly.

---

## Wiring

| Function | GPIO |
|---|---:|
| Motor PWM A / B / C | 16 / 17 / 5 |
| Bridge enable | 4 |
| AS5048A SCLK / MISO / MOSI | 18 / 19 / 23 |
| AS5048A CSn | 13 |
| Encoder power | 3.3 V, common GND |

- SPI mode 1, 1 MHz. PWM: LEDC at 20 kHz.
- 12 V supply, 7 pole pairs, fresh-board voltage limit 0.5 V.
- Saved NVS settings can override the default; the app/calibration CLI uses
  a 1.0 V torque request.
- Current sensing is disabled; the current-limit field is not measured
  current regulation.

See the [BOM and connection diagram](../hardware/README.md). These are signal
names, not a physical connector-order drawing. GPIO5 is motor PWM, not encoder CSn.

---

## Control

- **Encoder** (`src/platter_sensor.h`): one pipelined SPI frame per loop.
  Parity and error flag are checked on every frame; a bad frame holds the
  last good angle, and 16 in a row E-STOP the deck. Velocity is a
  least-squares fit over 8 ms.
- **Velocity loop:** P 0.08, I 0.4, Tf 5 ms (`config.h`, saved in NVS).
- **Cogging compensation:** up to 48 `(k, amp V, phase rad)` sinusoids in NVS
  key `harm`, summed into a 1024-bin table and added to the q-axis voltage
  by shaft angle. Identified by `tools/harmonic_compensate.py`.
- **Modes:** FREE and E-STOP disable the bridge; SPIN holds the target speed;
  BRAKE holds zero. E-STOP latches until FREE. More than 500 ms without an
  accepted command disables the bridge.
- **Cores:** core 1 runs the motor and encoder; core 0 runs Wi-Fi, UDP and the
  web server.

WebSocket actions used by the calibration tool:

| Action | Fields | Effect |
|---|---|---|
| `probe` | `slot` 0–2, `k`, `amp`, `phase` | Inject `amp·sin(k·θ + phase)` V; `k` 0 clears |
| `setHarmonic` | `slot` 0–47, `k`, `amp`, `phase` | Set one compensation sinusoid; `k` 0 clears |
| `clearHarmonics` | | Clear all |
| `saveConfig` | | Save tune and harmonics to NVS |

---

## Install

1. Disable motor output and close the Windows app.
2. For a new board, use `pio run -t upload --upload-port COMx` over its
   USB/serial programming connection. PlatformIO writes the bootloader,
   partition table and application. The application BIN alone is not a blank-flash image.
3. Join **PlatterDeck** (password in `config.h`). For an existing board,
   upload `firmware.bin` from its dashboard at `http://192.168.4.1`.
4. With output disabled, turn the shaft by hand to check the encoder. If the
   board is not calibrated, calibrate with the rotor free and save.
5. Run the cogging identification once (see the
   [setup guide](../docs/getting-started.md#4-identify-cogging-compensation-once-per-assembly)).

Settings live in NVS and survive firmware updates. Dashboard assets are
embedded during the build; no separate filesystem upload is required.

---

## UDP protocol

40-byte records, little endian, UDP port 19704. Telemetry packs one to four
records per datagram; commands use one.

| Bytes | Meaning |
|---|---|
| 0..4 | ASCII `PDW2` |
| 4 | 1 command, 2 telemetry |
| 5..8 | Zero |
| 8..12 | Host session ID |
| 12..16 | Sequence number |
| 16..20 | Board sample time, µs (commands echo it) |
| 20..24 | Board boot ID |
| 24..36 | 12-byte payload with CRC-8 |
| 36..40 | CRC-32 of bytes 0..36 |

- Telemetry payload: `AA`, seq, angle i32 (1/65536 rev), velocity i32
  (mrad/s), status, CRC-8.
- Command payload: `AA`, seq, mode (0 FREE, 1 SPIN, 2 BRAKE, 3 E-STOP), torque
  (0.02 V/unit), target velocity i32 (mrad/s), drag, two zeros, CRC-8.
- Status bits: 0 enabled, 1 encoder healthy, 2 driver/encoder/calibration
  unavailable, 3 link timeout, 4–5 actual mode.
- Discovery is FREE with boot ID 0. Later commands must echo a board
  timestamp under 500 ms old and the current boot ID; one peer owns the link.

---

## Build

```powershell
pio run                                   # .pio\build\esp32dev\firmware.bin
pio run -t upload --upload-port COMx
g++ -std=c++11 -Wall -Wextra -Werror test\test_wifi.cpp -o .pio\test_wifi.exe
.pio\test_wifi.exe                        # protocol test
```

SimpleFOC 2.3.2, espressif32 6.12.0 (Arduino core 2.0.17).

---

## Links

- [AS5048A/B datasheet (ams)](https://media.digikey.com/pdf/Data%20Sheets/Austriamicrosystems%20PDFs/AS5048A,B.pdf)
- [SimpleFOC](https://docs.simplefoc.com/)
- [Windows app](../platterdeck_windows_GUI-RUST/README.md) · [Project overview](../README.md)
