//! PlatterDeck payload: the 12-byte frames carried in bytes 24..36 of every
//! Wi-Fi packet (the envelope is in [`wifi`]). Shared units for the ESP32
//! firmware and the PC. Little-endian, `0xAA` sync, CRC-8 (poly 0x07,
//! init 0x00, no reflection; check value of `"123456789"` is `0xF4`).
//!
//! ## Board -> PC, 500 Hz ([`PlatterFrame`])
//!
//! | byte | field | unit |
//! |------|-------|------|
//! | 0 | `0xAA` | sync |
//! | 1 | `seq` | u8, +1 per frame, wraps |
//! | 2..6 | `angle` | i32, **1/65536 rev**, unwrapped multi-turn (wraps at +-32768 rev, PC unwraps) |
//! | 6..10 | `velocity` | i32, **milli-rad/s**, 12 ms least-squares angle regression on the board. 33 1/3 rpm = 3491 |
//! | 10 | `status` | bit flags, see [`status`] |
//! | 11 | `crc8` | over bytes 0..11 |
//!
//! ## PC -> board, 20 ms heartbeat + on change ([`HostFrame`])
//!
//! | byte | field | unit |
//! |------|-------|------|
//! | 0 | `0xAA` | sync |
//! | 1 | `seq` | u8 |
//! | 2 | `mode` | [`Mode`]: 0 FREE, 1 SPIN, 2 BRAKE, 3 ESTOP |
//! | 3 | `torque` | torque cap = SimpleFOC `voltage_limit`, **1/50 V** per LSB (50 = 1.0 V) |
//! | 4..8 | `target_vel` | i32 milli-rad/s, SPIN target. Sign = direction. 33 1/3 and 45 are presets on the PC, not modes |
//! | 8 | `drag` | FREE-mode viscous drag 0..255 (unused by the firmware, send 0) |
//! | 9..11 | reserved | 0 |
//! | 11 | `crc8` | over bytes 0..11 |
//!
//! The board drops to FREE (no torque) after [`wifi::TIMEOUT_US`] without an
//! accepted command and sets [`status::LINK_TIMEOUT`] until the next one.
#![no_std]
pub mod wifi;

pub const SYNC: u8 = 0xAA;
pub const FRAME_LEN: usize = 12;
/// Angle LSB: one revolution is 65536 units.
pub const ANGLE_UNITS_PER_REV: i64 = 65536;
/// 33 1/3 rpm in milli-rad/s.
pub const RPM33_MRAD: i32 = 3491;
/// 45 rpm in milli-rad/s.
pub const RPM45_MRAD: i32 = 4712;
/// Torque-cap byte resolution, volts per LSB.
pub const TORQUE_LSB_V: f32 = 0.02;

const TWO_PI: f32 = core::f32::consts::PI * 2.0;

/// CRC-8, poly 0x07, init 0x00, no reflection, no final xor.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
        }
    }
    crc
}

pub fn mrad_to_rpm(mrad: i32) -> f32 {
    mrad as f32 * 60.0 / (TWO_PI * 1000.0)
}

pub fn rpm_to_mrad(rpm: f32) -> i32 {
    (rpm * TWO_PI * 1000.0 / 60.0) as i32
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    Short,
    BadSync,
    BadCrc,
}

/// Status byte bits in [`PlatterFrame::status`].
pub mod status {
    /// Driver enabled, FOC running.
    pub const ENABLED: u8 = 1 << 0;
    /// Encoder frames arriving with good CRC.
    pub const SENSOR_OK: u8 = 1 << 1;
    /// DRV8313 nFT asserted since the last frame.
    pub const FAULT: u8 = 1 << 2;
    /// No accepted command for [`super::wifi::TIMEOUT_US`]; board forced FREE.
    pub const LINK_TIMEOUT: u8 = 1 << 3;
    /// Bits 4..6 carry the [`super::Mode`] actually running.
    pub const MODE_SHIFT: u8 = 4;
    pub const MODE_MASK: u8 = 0x03 << MODE_SHIFT;

    pub fn mode(status: u8) -> super::Mode {
        super::Mode::from_u8((status & MODE_MASK) >> MODE_SHIFT)
    }

    pub fn with_mode(status: u8, mode: super::Mode) -> u8 {
        (status & !MODE_MASK) | ((mode as u8) << MODE_SHIFT)
    }
}

/// Platter control mode requested by the PC.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Mode {
    /// Torque mode, target 0. Freewheel, FOC keeps tracking.
    #[default]
    Free = 0,
    /// Velocity mode, target from [`HostFrame::target_vel_mrad`].
    Spin = 1,
    /// Velocity mode, target 0.
    Brake = 2,
    /// Driver disabled.
    Estop = 3,
}

impl Mode {
    pub fn from_u8(v: u8) -> Mode {
        match v & 0x03 {
            0 => Mode::Free,
            1 => Mode::Spin,
            2 => Mode::Brake,
            _ => Mode::Estop,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Mode::Free => "FREE",
            Mode::Spin => "SPIN",
            Mode::Brake => "BRAKE",
            Mode::Estop => "ESTOP",
        }
    }
}

/// Board -> PC telemetry frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct PlatterFrame {
    pub seq: u8,
    /// Unwrapped shaft angle, 1/65536 rev.
    pub angle: i32,
    /// Filtered shaft velocity, milli-rad/s.
    pub velocity_mrad: i32,
    pub status: u8,
}

impl PlatterFrame {
    pub fn encode(&self) -> [u8; FRAME_LEN] {
        let mut f = [0u8; FRAME_LEN];
        f[0] = SYNC;
        f[1] = self.seq;
        f[2..6].copy_from_slice(&self.angle.to_le_bytes());
        f[6..10].copy_from_slice(&self.velocity_mrad.to_le_bytes());
        f[10] = self.status;
        f[11] = crc8(&f[..11]);
        f
    }

    pub fn decode(b: &[u8]) -> Result<Self, FrameError> {
        check_envelope(b)?;
        Ok(PlatterFrame {
            seq: b[1],
            angle: i32::from_le_bytes([b[2], b[3], b[4], b[5]]),
            velocity_mrad: i32::from_le_bytes([b[6], b[7], b[8], b[9]]),
            status: b[10],
        })
    }

    pub fn rpm(&self) -> f32 {
        mrad_to_rpm(self.velocity_mrad)
    }
}

/// PC -> board command frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct HostFrame {
    pub seq: u8,
    pub mode: Mode,
    /// Torque cap, [`TORQUE_LSB_V`] per LSB.
    pub torque: u8,
    /// SPIN target, milli-rad/s.
    pub target_vel_mrad: i32,
    pub drag: u8,
}

impl HostFrame {
    pub fn encode(&self) -> [u8; FRAME_LEN] {
        let mut f = [0u8; FRAME_LEN];
        f[0] = SYNC;
        f[1] = self.seq;
        f[2] = self.mode as u8;
        f[3] = self.torque;
        f[4..8].copy_from_slice(&self.target_vel_mrad.to_le_bytes());
        f[8] = self.drag;
        f[11] = crc8(&f[..11]);
        f
    }

    pub fn decode(b: &[u8]) -> Result<Self, FrameError> {
        check_envelope(b)?;
        Ok(HostFrame {
            seq: b[1],
            mode: Mode::from_u8(b[2]),
            torque: b[3],
            target_vel_mrad: i32::from_le_bytes([b[4], b[5], b[6], b[7]]),
            drag: b[8],
        })
    }

    pub fn torque_volts(&self) -> f32 {
        self.torque as f32 * TORQUE_LSB_V
    }

    pub fn torque_from_volts(v: f32) -> u8 {
        let lsb = v / TORQUE_LSB_V;
        if lsb <= 0.0 {
            0
        } else if lsb >= 255.0 {
            255
        } else {
            (lsb + 0.5) as u8
        }
    }
}

fn check_envelope(b: &[u8]) -> Result<(), FrameError> {
    if b.len() < FRAME_LEN {
        return Err(FrameError::Short);
    }
    if b[0] != SYNC {
        return Err(FrameError::BadSync);
    }
    if crc8(&b[..11]) != b[11] {
        return Err(FrameError::BadCrc);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn crc8_check_value() {
        assert_eq!(crc8(b"123456789"), 0xF4);
    }

    #[test]
    fn platter_roundtrip() {
        let f = PlatterFrame {
            seq: 7,
            angle: -123_456,
            velocity_mrad: 3491,
            status: status::ENABLED | status::SENSOR_OK,
        };
        let b = f.encode();
        assert_eq!(b[0], SYNC);
        assert_eq!(PlatterFrame::decode(&b), Ok(f));
        let mut bad = b;
        bad[5] ^= 1;
        assert_eq!(PlatterFrame::decode(&bad), Err(FrameError::BadCrc));
        assert_eq!(PlatterFrame::decode(&b[..11]), Err(FrameError::Short));
    }

    #[test]
    fn host_roundtrip() {
        let f = HostFrame { seq: 200, mode: Mode::Spin, torque: 50, target_vel_mrad: RPM33_MRAD, drag: 3 };
        assert_eq!(HostFrame::decode(&f.encode()), Ok(f));
        assert_eq!(HostFrame::torque_from_volts(1.0), 50);
        assert_eq!(HostFrame::torque_from_volts(-1.0), 0);
        assert_eq!(HostFrame::torque_from_volts(99.0), 255);
        let t = HostFrame { torque: 50, ..Default::default() };
        assert!((t.torque_volts() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn units() {
        assert!((mrad_to_rpm(RPM33_MRAD) - 33.333).abs() < 0.01);
        assert!((mrad_to_rpm(RPM45_MRAD) - 45.0).abs() < 0.01);
        assert!((rpm_to_mrad(33.3333) - RPM33_MRAD).abs() <= 1);
    }

    #[test]
    fn status_mode_bits() {
        let s = status::with_mode(status::ENABLED, Mode::Brake);
        assert_eq!(status::mode(s), Mode::Brake);
        assert_ne!(s & status::ENABLED, 0);
    }
}
