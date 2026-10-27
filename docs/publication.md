# Publication plan

Prepared 2026-10-28. Publish the working v1 code and document the build before
adding features. The first release can be a working prototype with clearly
listed reproduction gaps; a magazine build article needs those gaps filled.

Source repository: [nzwildcode/esp-foc-platter-deck](https://github.com/nzwildcode/esp-foc-platter-deck).

## Recommended order

1. Upload both code sets, the MIT licence, setup guide, wiring and BOM to GitHub.
   Keep the current motor and audio behaviour as the release baseline.
2. Add photographs of the actual deck, board labels, encoder and mounting;
   a desktop screenshot; and a 30-60 second playback/scratch demonstration.
3. Publish an Elektor Labs project and pitch the motor-control story. Elektor
   explicitly prefers Labs for article/project proposals. A published Labs
   project is eligible for editorial consideration, with no promised review
   time. First-time posters may need manual approval.
4. Use a short Reddit demo to find builders and get practical feedback,
   linking the same repository. Check the chosen community's live rules
   before posting; the public rules pages did not expose their rules in this check.

Elektor is the main route for the desired article. Reddit is useful for
early feedback and reach; neither posting route guarantees a result.
Keep the full magazine manuscript for discussion with the editor. Do not
claim an exclusivity rule that has not been established.

Sources checked 2026-10-28: [Elektor submission page](https://www.elektormagazine.com/elektor-submission-page)
and [Labs user manual](https://www.elektormagazine.com/news/elektor-labs-user-manual).
The submission page links the [Author's Guide](https://www.elektormagazine.com/files/attachment/44777);
use that guide and the editor's current requirements for the eventual manuscript.

## Elektor Labs draft

Status: draft only; add actual project media before posting.

**Title:** Build an ESP32 Motorised DJ Platter with Cogging Compensation

**Teaser:** A gimbal motor, magnetic encoder and ESP32 FOC board drive a scratchable DJ platter, with a Windows audio deck linked over Wi-Fi.

PlatterDeck is a working motorised controller for digital audio. An ESP32
FOC board drives a GM2804 gimbal motor and reads an AS5048A magnetic encoder.
It streams timestamped platter motion to a Windows application. STEADY mode
plays at the selected tempo; VINYL mode follows hand movement, including reverse.

The engineering focus is low-speed motor ripple. The firmware uses SimpleFOC,
an encoder velocity estimator and an angle-based feed-forward table. A Python
tool probes the running motor and identifies compensation harmonics for the
particular assembly. The PC blends slow free-running and fast hand-motion
velocity estimates to control audio playback.

The first version is working on the prototype. Firmware, Windows source,
setup instructions and a BOM are provided in the
[source repository](https://github.com/nzwildcode/esp-foc-platter-deck). It is a single-deck,
Windows-based build. The remaining documentation work covers exact platter
mechanics, supplier details and reproducible measurement captures.

The article would explain the hardware, wiring, motor tuning, identification
method and the distinction between measured speed ripple, software audio-rate
variation and acoustic performance. Numeric results will be accompanied by
their test conditions and raw captures before being presented as final measurements.

**Attach:** repository/release link, deck photo, annotated wiring, BOM, GUI
screenshot, demo video and author name/bio. Use original project photographs.

## Reddit draft

Status: draft only; add a real demonstration before posting.

**Title:** I built an ESP32 motorised DJ platter using a gimbal motor and magnetic encoder

The first version of PlatterDeck is working: a GM2804 gimbal motor, AS5048A
encoder and ESP32 FOC board connected over Wi-Fi to a Windows audio app.
STEADY mode plays at a fixed tempo; VINYL mode follows the platter so you can
stop, reverse and scratch the track by hand.

The most interesting part was reducing low-speed motor ripple. I used
per-motor cogging compensation and separate velocity estimates for free
rotation and hand movement. The board handles motor control; audio stays
on the PC.

Demo: [insert video link]

Code, wiring and BOM: https://github.com/nzwildcode/esp-foc-platter-deck

It is currently one deck and Windows only. I would welcome feedback on the
build instructions and how the platter responds during scratching.

## Material needed next

- Complete BOM purchase links/prices and actual supply rating.
- Record platter material, dimensions, mass and mounting hardware.
- Photograph the working assembly and label connector orientation.
- Capture a short demo with audio you have permission to share.
- Save before/after speed logs and a reproducible analysis; measure actual
  motion-to-sound delay separately from the configured 40 ms playback delay.
- Supply a short author bio and the name to use on the article.

No article, email or social post has been submitted by this preparation.
