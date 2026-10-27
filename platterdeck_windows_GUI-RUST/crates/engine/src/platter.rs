//! Latest platter sample, lock-free. Written by the Wi-Fi link (500 Hz) or the
//! simulator (~1 kHz), read by the audio callback and the UI.
use platter_proto::ANGLE_UNITS_PER_REV;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicI64, AtomicU32, AtomicU64, AtomicU8, Ordering};

pub struct PlatterState {
    angle: AtomicI64,
    vel: AtomicI32,
    t_us: AtomicU64,
    status: AtomicU8,
    frames: AtomicU64,
    crc_errors: AtomicU32,
    dropped: AtomicU32,
    connected: AtomicBool,
    rejected: AtomicU32,
    delay_us: AtomicU32,
    max_gap_us: AtomicU32,
    wifi: AtomicBool,
    /// 256 samples: 512 ms at the 500 Hz Wi-Fi rate, enough for the
    /// deck's 160 ms flywheel window and the UI's 250 ms readout.
    history: [HistorySample; 256],
}

#[derive(Default)]
struct HistorySample {
    stamp: AtomicU64,
    angle: AtomicI64,
    velocity: AtomicI32,
    time: AtomicU64,
    status: AtomicU8,
}

#[derive(Clone, Copy, Default)]
pub struct TimedSample {
    pub angle: i64,
    pub velocity: i32,
    pub time: u64,
    pub status: u8,
}

impl Default for PlatterState {
    fn default() -> Self {
        Self { angle: AtomicI64::new(0), vel: AtomicI32::new(0), t_us: AtomicU64::new(0),
            status: AtomicU8::new(0), frames: AtomicU64::new(0), crc_errors: AtomicU32::new(0),
            dropped: AtomicU32::new(0), connected: AtomicBool::new(false), rejected: AtomicU32::new(0),
            delay_us: AtomicU32::new(0), max_gap_us: AtomicU32::new(0), wifi: AtomicBool::new(false),
            history: std::array::from_fn(|_| HistorySample::default()) }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PlatterSnapshot {
    /// Unwrapped angle, 1/65536 rev (i64: no wrap in practice).
    pub angle_units: i64,
    pub vel_mrad: i32,
    /// `clock::now_us()` when the sample arrived.
    pub t_us: u64,
    pub status: u8,
    pub connected: bool,
    pub frames: u64,
    pub crc_errors: u32,
    pub dropped: u32,
    pub rejected: u32,
    pub delay_us: u32,
    pub max_gap_us: u32,
    pub wifi: bool,
}

impl PlatterSnapshot {
    pub fn angle_rev(&self) -> f64 {
        self.angle_units as f64 / ANGLE_UNITS_PER_REV as f64
    }
    pub fn vel_rad_s(&self) -> f64 {
        self.vel_mrad as f64 * 1e-3
    }
    pub fn rpm(&self) -> f64 {
        self.vel_rad_s() * 60.0 / std::f64::consts::TAU
    }
}

impl PlatterState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Publish one sample. `t_us` is the arrival time from [`crate::clock::now_us`].
    pub fn publish(&self, angle_units: i64, vel_mrad: i32, status: u8, t_us: u64) {
        let id = self.frames.load(Ordering::Relaxed) + 1;
        let slot = &self.history[id as usize % self.history.len()];
        // Sequentially consistent slot stamps make the multi-field sample
        // coherent, including if the writer wraps the ring during a read.
        slot.stamp.store(id * 2 - 1, Ordering::SeqCst);
        slot.angle.store(angle_units, Ordering::SeqCst);
        slot.velocity.store(vel_mrad, Ordering::SeqCst);
        slot.time.store(t_us, Ordering::SeqCst);
        slot.status.store(status, Ordering::SeqCst);
        slot.stamp.store(id * 2, Ordering::SeqCst);
        self.angle.store(angle_units, Ordering::Relaxed);
        self.vel.store(vel_mrad, Ordering::Relaxed);
        self.status.store(status, Ordering::Relaxed);
        // Release on the timestamp orders the three stores above before it.
        self.t_us.store(t_us, Ordering::Release);
        self.frames.store(id, Ordering::Release);
    }

    pub fn set_wifi(&self, wifi: bool) { self.wifi.store(wifi, Ordering::Relaxed); }

    fn sample(&self, id: u64) -> Option<TimedSample> {
        if id == 0 { return None; }
        let slot = &self.history[id as usize % self.history.len()];
        let expected = id * 2;
        if slot.stamp.load(Ordering::SeqCst) != expected { return None; }
        let s = TimedSample { angle: slot.angle.load(Ordering::SeqCst),
            velocity: slot.velocity.load(Ordering::SeqCst), time: slot.time.load(Ordering::SeqCst),
            status: slot.status.load(Ordering::SeqCst) };
        if slot.stamp.load(Ordering::SeqCst) == expected { Some(s) } else { None }
    }

    /// Timestamp interpolation removes packet-arrival timing from playback.
    /// No allocations, waits, or locks on the audio thread.
    pub fn angle_at(&self, time: u64) -> Option<f64> {
        let id = self.frames.load(Ordering::Acquire);
        let mut newer = self.sample(id)?;
        if time >= newer.time {
            let dt = time.saturating_sub(newer.time).min(12_000) as f64 * 1e-6;
            let mut rev_per_sec = newer.velocity as f64 * 0.001 / std::f64::consts::TAU;
            // A brief packet gap uses angle-derived velocity across several
            // encoder samples, not a noisy instantaneous speed field.
            for offset in 1..(self.history.len() as u64).min(id) {
                let Some(older) = self.sample(id - offset) else { break; };
                let span = newer.time.saturating_sub(older.time);
                if span > 30_000 { break; }
                if span >= 8_000 {
                    rev_per_sec = (newer.angle - older.angle) as f64 /
                        ANGLE_UNITS_PER_REV as f64 * 1e6 / span as f64;
                    break;
                }
            }
            return Some(newer.angle as f64 / ANGLE_UNITS_PER_REV as f64 +
                rev_per_sec * dt);
        }
        for offset in 1..(self.history.len() as u64).min(id) {
            let Some(older) = self.sample(id - offset) else { break; };
            // A reconnect/source switch is a new time segment, not a ramp.
            if newer.time <= older.time || newer.time - older.time > 100_000 { break; }
            if time >= older.time {
                let f = (time - older.time) as f64 / (newer.time - older.time) as f64;
                return Some((older.angle as f64 + (newer.angle - older.angle) as f64 * f) /
                    ANGLE_UNITS_PER_REV as f64);
            }
            newer = older;
        }
        Some(newer.angle as f64 / ANGLE_UNITS_PER_REV as f64)
    }

    /// Least-squares platter velocity (rev/s) over the `window_us` of
    /// captured samples ending at `time`, or at the newest sample when
    /// `time` is later. Encoder quantisation and per-sample timing noise
    /// average out instead of stepping the audio rate every block. None
    /// until the window holds at least three samples spanning a third of it.
    pub fn velocity_at(&self, time: u64, window_us: u64) -> Option<f64> {
        let id = self.frames.load(Ordering::Acquire);
        let newest = self.sample(id)?;
        let end = time.min(newest.time);
        let start = end.saturating_sub(window_us);
        let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0u32, 0f64, 0f64, 0f64, 0f64);
        let (mut first, mut last) = (u64::MAX, 0u64);
        let mut previous = newest.time;
        for offset in 0..(self.history.len() as u64).min(id) {
            let Some(s) = self.sample(id - offset) else { break; };
            // A reconnect/source switch is a new time segment, not a ramp.
            if offset > 0 && (s.time >= previous || previous - s.time > 100_000) { break; }
            previous = s.time;
            if s.time > end { continue; }
            if s.time < start { break; }
            let x = (s.time as f64 - end as f64) * 1e-6;
            let y = (s.angle - newest.angle) as f64 / ANGLE_UNITS_PER_REV as f64;
            n += 1; sx += x; sy += y; sxx += x * x; sxy += x * y;
            first = first.min(s.time); last = last.max(s.time);
        }
        if n < 3 || last.saturating_sub(first) < window_us / 3 { return None; }
        let denom = n as f64 * sxx - sx * sx;
        if denom <= 1e-18 { return None; }
        Some((n as f64 * sxy - sx * sy) / denom)
    }

    pub fn set_stats(&self, crc_errors: u32, dropped: u32) {
        self.crc_errors.store(crc_errors, Ordering::Relaxed);
        self.dropped.store(dropped, Ordering::Relaxed);
    }

    /// Ignore encoder chatter at rest, but retain slow deliberate hand motion.
    /// Requires at least 30 ms of continuous capture history within 8 counts
    /// of the 14-bit AS5048A (32 protocol units).
    pub fn stationary(&self) -> bool {
        let id = self.frames.load(Ordering::Acquire);
        let Some(latest) = self.sample(id) else { return false; };
        let (mut lo, mut hi) = (latest.angle, latest.angle);
        let mut previous = latest.time;
        for offset in 1..(self.history.len() as u64).min(id) {
            let Some(s) = self.sample(id - offset) else { return false; };
            if s.time >= previous || previous - s.time > 10_000 { return false; }
            previous = s.time;
            lo = lo.min(s.angle); hi = hi.max(s.angle);
            if hi - lo > 32 { return false; }
            if latest.time - s.time >= 30_000 { return true; }
        }
        false
    }

    pub fn set_connected(&self, on: bool) {
        self.connected.store(on, Ordering::Relaxed);
    }

    pub fn set_wifi_stats(&self, rejected: u32, delay_us: u32, max_gap_us: u32) {
        self.rejected.store(rejected, Ordering::Relaxed);
        self.delay_us.store(delay_us, Ordering::Relaxed);
        self.max_gap_us.store(max_gap_us, Ordering::Relaxed);
    }

    pub fn snapshot(&self) -> PlatterSnapshot {
        let frames = self.frames.load(Ordering::Acquire);
        let sample = self.sample(frames).or_else(|| self.sample(frames.saturating_sub(1))).unwrap_or_default();
        PlatterSnapshot {
            angle_units: sample.angle,
            vel_mrad: sample.velocity,
            t_us: sample.time,
            status: sample.status,
            connected: self.connected.load(Ordering::Relaxed),
            frames,
            crc_errors: self.crc_errors.load(Ordering::Relaxed),
            dropped: self.dropped.load(Ordering::Relaxed),
            rejected: self.rejected.load(Ordering::Relaxed),
            delay_us: self.delay_us.load(Ordering::Relaxed),
            max_gap_us: self.max_gap_us.load(Ordering::Relaxed),
            wifi: self.wifi.load(Ordering::Relaxed),
        }
    }
}
