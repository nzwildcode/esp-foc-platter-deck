# First public release checklist

Preparation date: 2026-10-28. Working source baseline: `23e4191`.
This preparation changes documentation and packaging, not firmware or app behaviour.
The Rust workspace version remains `0.1.0`; a release tag has not been created.

## Prepared

- [x] Standard root README linking both code sets.
- [x] Existing MIT licence retained with its current attribution.
- [x] BOM in CSV and readable hardware list, with unconfirmed fields marked.
- [x] Signal-level wiring diagram and GPIO table derived from source.
- [x] First-flash, build, calibration and playback instructions.
- [x] Elektor Labs and Reddit drafts; no submissions made.

The local source package is `dist/PlatterDeck-source-2026-10-28.zip`; its
publishing checkout is `dist/github-upload/`. These local output folders are
not included on GitHub. Packaging includes both code sets and
documentation, with per-file SHA-256 checksums. It excludes the earlier EXE/BIN,
Git history, build outputs, local research and mechanical drafts. No source
commit or public release is implied by the package date.

## Software checks

Run on Windows on 2026-10-28, without uploading firmware or driving hardware:

| Check | Result |
|---|---|
| `cargo test --workspace --locked` | 27 tests passed |
| `cargo build --release --workspace --locked` | Windows app and diagnostics CLI built successfully |
| `pio run` | Successful ESP32 firmware build |
| C++ Wi-Fi protocol test, `g++ -std=c++11 -Wall -Wextra -Werror` | All checks passed |

Toolchain: Rust 1.97.1, Cargo 1.97.1, PlatformIO Core 6.2.0, Python 3.11.0.
Local Markdown file links and all 15 BOM rows were checked. Tracked source,
build configuration and existing binaries match the working baseline.

The firmware build resolved ArduinoJson 7.4.3 under the existing `^7.3.1`
constraint. Other pinned libraries: SimpleFOC 2.3.2, ESPAsyncWebServer 3.6.0
and AsyncTCP 3.3.2; platform espressif32 6.12.0.

These checks do not establish physical fit, motor performance or acoustic
latency. The working prototype is reported by the builder; no new hardware
test was performed during this cleanup.

## GitHub publication

- [x] Repository created with public visibility and write access verified:
  [nzwildcode/esp-foc-platter-deck](https://github.com/nzwildcode/esp-foc-platter-deck).
- [x] Review the public snapshot: include firmware and app source, Cargo.lock,
  dashboard, tests, calibration tools, docs and licence. Exclude build caches,
  calibration logs, local `Elektor/` research and unverified mechanical drafts.
- [x] Preserve the original local history separately. The public repository
  starts with a clean source snapshot; older editorial working notes are excluded.
- [ ] Push the reviewed snapshot and confirm that the README and both code sets
  are visible. Record the public URL here.

## Before tagging and distributing binaries

- [ ] Choose the release tag (for example `v0.1.0`, matching Cargo metadata).
- [ ] Record the final source commit, toolchain versions and SHA-256 hashes.
- [ ] Rebuild the app, CLI and firmware from that commit and run a physical
  smoke test: encoder, calibration, PLAY/PAUSE, reverse scratch, E-STOP/FREE,
  disconnect timeout and reconnect.
- [ ] Keep provenance clear: the existing root EXE and firmware BIN are earlier
  prototype artifacts; a successful source build alone does not prove they
  came from the final release commit.
- [ ] Include relevant third-party licence notices with binary downloads.
- [ ] Offer app/CLI and firmware assets with their setup instructions; the
  application BIN alone is insufficient to initialise a blank ESP32.

## Before describing the build as reproducible

- [ ] Complete actual supply rating/polarity and board/motor purchase links.
- [ ] Document platter, mount and fastener dimensions; verify any published CAD.
- [ ] Add an annotated photograph showing physical connector orientation.
- [ ] Add raw performance captures, method and conditions for numeric claims.
- [ ] Add project photos, desktop screenshot, short demo and author bio.

See [publication.md](publication.md) for the Elektor/Reddit route and drafts.
