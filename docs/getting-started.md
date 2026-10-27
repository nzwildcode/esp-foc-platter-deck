# Build and first use

Use Windows for the desktop app and calibration tools. Install Rust with
the Windows MSVC build tools, PlatformIO, and Python 3. The commands below
start in the repository root. Download build dependencies while your PC
still has internet access; the board's hotspot does not provide internet.

## 1. Build the desktop app and diagnostics CLI

```powershell
Set-Location platterdeck_windows_GUI-RUST
cargo test --workspace --locked
cargo build --release --workspace --locked
python -m pip install numpy websocket-client
Set-Location ..
```

Outputs are `platterdeck_windows_GUI-RUST/target/release/platterdeck.exe`
and `platter-cli.exe`. The calibration script expects the CLI at that path.

You can try the app without hardware using:

```powershell
.\platterdeck_windows_GUI-RUST\target\release\platterdeck.exe --sim
```

## 2. Build and flash a new board

Check the [wiring](../hardware/README.md) and programming interface first.
Use the board's USB/serial programming connection with the correct boot mode.
Replace `COMx` with the port shown on your PC.

```powershell
Set-Location PlatterDeck_FOC
pio run
pio run -t upload --upload-port COMx
Set-Location ..
```

PlatformIO uploads the bootloader, partition table and application at their
configured offsets. Do not flash the standalone `firmware.bin` at address
zero. The web assets are embedded by `embed_web.py`; no `uploadfs` step is needed.

The build currently pins espressif32 6.12.0, SimpleFOC 2.3.2,
ESPAsyncWebServer 3.6.0 and AsyncTCP 3.3.2. ArduinoJson is specified as
`^7.3.1`; record the resolved version when preparing binaries.

## 3. Check the encoder and calibrate FOC

1. Close the Windows deck app and other motor controllers.
2. Connect to **PlatterDeck**, password **12345678**. These defaults are in
   `PlatterDeck_FOC/src/config.h`; change them there and rebuild if needed.
3. Open `http://192.168.4.1`. Keep motor output disabled and turn the rotor
   by hand to confirm a healthy encoder reading.
4. Check the supply setting is 12 V and the motor has 7 pole pairs. A fresh
   board defaults to a 0.5 V motor voltage limit. Existing saved settings survive
   updates, so inspect them rather than assuming they reset.
5. With the motor securely mounted and rotor free to move, select **Calibrate FOC**.
   Calibration moves the rotor. Wait for success, then select **Save settings**.
   Do not proceed if calibration or encoder health fails.

## 4. Identify cogging compensation once per assembly

Use one motor controller at a time. Close the desktop app and avoid sending
dashboard motor commands while the calibration script is running. Keep the
rotor unobstructed and monitor the first run for abnormal heating or motion.
The script clears existing harmonics, spins the motor with a 1.0 V torque
request and saves replacement harmonics to NVS at completion. It typically
takes about 20-25 minutes.

From the repository root:

```powershell
Set-Location platterdeck_windows_GUI-RUST
New-Item -ItemType Directory -Force -Path calibration-v1 | Out-Null
python tools/harmonic_compensate.py calibration-v1
Set-Location ..
```

Keep the generated logs with your build notes. Repeat identification after
changing the motor, magnet or platter. A previously calibrated working deck
does not need to be recalibrated just for this documentation cleanup.

## 5. Play

Keep the PC connected to the board hotspot and use the Wi-Fi adapter's
Maximum Performance power setting where available. Start the desktop app,
load a local track, select STEADY or VINYL, and press PLAY.
After E-STOP, select FREE MOTOR before trying PLAY again.

If the board does not connect, check `192.168.4.1:19704`, Windows firewall
permission for the app and that another app/CLI does not own the link.
If hand movement produces rough audio, confirm calibration, compensation
and Wi-Fi performance before changing control gains.

## Updating an existing board

Disable output and close the desktop app. On the existing dashboard, upload
the application `firmware.bin` and wait for the reboot. Saved settings remain
in NVS. For source-built firmware the file is
`PlatterDeck_FOC/.pio/build/esp32dev/firmware.bin`.
