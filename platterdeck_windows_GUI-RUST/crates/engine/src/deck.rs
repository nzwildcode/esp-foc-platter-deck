//! The scratch engine. Runs inside the audio callback: no locks, no
//! allocation, no logging.
//!
//! VINYL: the platter is the timeline. One revolution at the selected
//! platter speed is `60 / rpm` seconds of audio (1.8 s at 33 1/3), so
//! `playhead = anchor_pos + (angle - anchor_rev) * samples_per_rev`.
//! The playhead follows the platter through a tracking loop instead of
//! stepping to the raw encoder angle every block: the rate is a
//! least-squares platter velocity plus a slow position correction, and
//! it ramps linearly across every block. Two velocity windows are fitted
//! over the timestamped encoder samples. While the platter runs free at
//! the commanded speed the audio follows the 160 ms one (the flywheel:
//! encoder noise and the residual motor ripple above ~5 Hz average out,
//! under 0.3 % of pitch gets through). A hand on the platter shows as
//! the 24 ms velocity leaving the 160 ms one or the commanded speed, and
//! tracking switches to the fast window at once, blending back 300 ms
//! after the platter settles. The position error stays within a few
//! milliseconds of audio either way and cannot accumulate. Cubic
//! (Catmull-Rom) interpolation reads the track at the fractional playhead.
//!
//! STEADY: the audio clock is the timeline and the platter only gates
//! playback (SPIN running, link up, no fault). 33/45 selects the platter
//! speed; the music tempo comes from the pitch fader alone.
use crate::clock::now_us;
use crate::decode::Track;
use crate::platter::PlatterState;
use crate::tap::{Event, Tap};
use crossbeam_channel::{Receiver, Sender};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use platter_proto::{status, Mode};

/// Seconds of audio per platter revolution at 33 1/3 rpm.
pub const SECONDS_PER_REV: f64 = 60.0 / (100.0 / 3.0);
/// Default platter speed, rpm.
pub const DEFAULT_RPM: f64 = 100.0 / 3.0;
/// Platter samples older than this are treated as a stopped platter. The
/// link layer declares the link down at the same age.
pub const STALE_US: u64 = 500_000;
/// Wi-Fi playback delay: the platter time that reaches the DAC now is this
/// far behind the wall clock, so sample batching (8 ms), transit and the
/// output ring buffer (20 ms) are covered by interpolation, not guesswork.
/// This is the scratch latency, platter motion to loudspeaker.
pub const WIFI_BUFFER_US: u64 = 40_000;
/// Fast least-squares velocity window for hands-on tracking: 12 encoder
/// samples at 500 Hz, about one cogging period of the 12N14P GM2804 at
/// 33 rpm (84 cycles/rev, 22 ms). About 2 % noisy with the AS5048A.
pub const VEL_WINDOW_US: u64 = 24_000;
/// Flywheel velocity window while the platter runs free: 80 samples.
/// Attenuates the 14x/rev ripple (7.8 Hz at 33 rpm) 10x and everything
/// above 15 Hz by 50x or more; encoder noise contributes ~0.03 %.
pub const FLYWHEEL_WINDOW_US: u64 = 160_000;
/// Position-correction time constant while hands-on. Drift from the
/// platter angle closes with this time constant; one encoder count
/// (5.3 samples at 48 kHz) then perturbs the rate by about 0.1 %.
pub const TRACK_TAU_S: f64 = 0.1;
/// Position-correction time constant on the flywheel: encoder noise and
/// the ripple's position wobble (10-30 samples) then move the rate by
/// under 0.1 %.
pub const FLYWHEEL_TAU_S: f64 = 0.3;
/// A position error beyond this (5 ms of audio: glitch, re-anchor, hard
/// scratch) is closed at the fast time constant, not the flywheel's.
pub const POS_ERR_HANDS: f64 = 240.0;
/// Hands-on when the fast velocity leaves the flywheel one by this
/// fraction of the nominal platter speed (5 sigma of the fast estimate),
/// or leaves the commanded speed by [`HANDS_CMD_FRAC`].
pub const HANDS_ON_FRAC: f64 = 0.12;
pub const HANDS_CMD_FRAC: f64 = 0.15;
/// The flywheel returns this long after the platter last looked disturbed
/// (half the thresholds above), blending over [`HANDS_BLEND_S`].
pub const HANDS_OFF_US: u64 = 300_000;
pub const HANDS_BLEND_S: f64 = 0.15;
/// Without new samples the last velocity is held this long, then decays
/// with [`DECAY_TAU_US`]: a stalled link neither freezes nor runs away.
pub const HOLD_US: u64 = 100_000;
pub const DECAY_TAU_US: f64 = 200_000.0;
/// Extrapolation beyond the newest sample never exceeds this.
pub const MAX_EXTRAP_US: u64 = 60_000;
/// Output latency assumption never exceeds this.
pub const MAX_LEAD_US: u64 = 80_000;
/// Playback rate cap relative to nominal. Beyond it the playhead snaps.
pub const RATE_CAP: f64 = 32.0;

pub enum DeckCommand {
    /// Replace the track (None = eject). Playhead re-anchors at sample 0.
    Load(Option<Arc<Track>>),
    /// Needle drop: re-anchor the current platter angle at this sample.
    Jump(f64),
    /// Output gain, linear.
    Gain(f32),
    /// Extra lead added to the measured output latency, microseconds.
    ExtraLeadUs(f64),
    /// Use the audio clock at the requested tempo instead of encoder angle.
    Steady(bool),
    Transport(Mode),
    /// STEADY tempo relative to the original track (1.0 = pitch 0).
    Tempo(f64),
    /// Platter speed the track is "cut" for: 33 1/3 or 45 rpm.
    PlatterRpm(f64),
}

/// UI readback, written by the callback.
#[derive(Default)]
pub struct DeckReadback {
    pos_bits: AtomicU64,
    rate_bits: AtomicU64,
    pub block_frames: AtomicU32,
    pub out_rate: AtomicU32,
    pub callbacks: AtomicU64,
    /// Platter-to-DAC delay for this block: the Wi-Fi playback delay, or the
    /// output latency when that is larger or for non-Wi-Fi sources.
    pub lead_us: AtomicU32,
    peak_l_bits: AtomicU32,
    peak_r_bits: AtomicU32,
}

impl DeckReadback {
    /// Playhead in track samples.
    pub fn pos(&self) -> f64 {
        f64::from_bits(self.pos_bits.load(Ordering::Relaxed))
    }
    /// Peak |sample| of the last block after gain, left and right. For the meter.
    pub fn peaks(&self) -> (f32, f32) {
        (
            f32::from_bits(self.peak_l_bits.load(Ordering::Relaxed)),
            f32::from_bits(self.peak_r_bits.load(Ordering::Relaxed)),
        )
    }
    fn set_peaks(&self, l: f32, r: f32) {
        self.peak_l_bits.store(l.to_bits(), Ordering::Relaxed);
        self.peak_r_bits.store(r.to_bits(), Ordering::Relaxed);
    }
    /// Playback rate at the end of the last block relative to nominal
    /// (1.0 = the track at its original tempo).
    pub fn rate(&self) -> f64 {
        f64::from_bits(self.rate_bits.load(Ordering::Relaxed))
    }
}

pub struct Deck {
    rx: Receiver<DeckCommand>,
    trash: Sender<Arc<Track>>,
    platter: Arc<PlatterState>,
    rb: Arc<DeckReadback>,
    track: Option<Arc<Track>>,
    pos: f64,
    /// Track samples per output frame right now.
    rate: f64,
    anchor_pos: f64,
    anchor_rev: f64,
    gain: f32,
    gain_now: f32,
    extra_lead_us: f64,
    platter_rpm: f64,
    samples_per_rev: f64,
    tempo: f64,
    timeline_angle: f64,
    timeline_valid: bool,
    timeline_wifi: bool,
    steady: bool,
    transport: Mode,
    motion_gain: f32,
    /// 1 = fast hands-on tracking, 0 = flywheel; blends in between.
    hands: f64,
    /// Last time the platter looked disturbed.
    hands_since: u64,
    tap: Option<Arc<Tap>>,
}

impl Deck {
    pub fn new(
        rx: Receiver<DeckCommand>,
        trash: Sender<Arc<Track>>,
        platter: Arc<PlatterState>,
        rb: Arc<DeckReadback>,
        tap: Option<Arc<Tap>>,
    ) -> Self {
        Deck {
            rx,
            trash,
            platter,
            rb,
            tap,
            track: None,
            pos: 0.0,
            rate: 0.0,
            anchor_pos: 0.0,
            anchor_rev: 0.0,
            gain: 1.0,
            gain_now: 0.0,
            extra_lead_us: 0.0,
            platter_rpm: DEFAULT_RPM,
            samples_per_rev: 1.0,
            tempo: 1.0,
            timeline_angle: 0.0,
            timeline_valid: false,
            timeline_wifi: false,
            steady: false,
            transport: Mode::Free,
            motion_gain: 0.0,
            hands: 1.0,
            hands_since: 0,
        }
    }

    fn update_samples_per_rev(&mut self) {
        if let Some(t) = &self.track {
            self.samples_per_rev = t.sample_rate as f64 * 60.0 / self.platter_rpm.max(1.0);
        }
    }

    fn drain_commands(&mut self, angle_rev_now: f64) {
        while let Ok(cmd) = self.rx.try_recv() {
            match cmd {
                DeckCommand::Load(t) => {
                    if let Some(old) = self.track.take() {
                        // Free the old buffer off this thread.
                        let _ = self.trash.try_send(old);
                    }
                    self.track = t;
                    self.update_samples_per_rev();
                    self.pos = 0.0;
                    self.anchor_pos = 0.0;
                    self.anchor_rev = angle_rev_now;
                }
                DeckCommand::Jump(sample) => {
                    self.pos = sample;
                    self.anchor_pos = sample;
                    self.anchor_rev = angle_rev_now;
                }
                DeckCommand::Gain(g) => self.gain = g,
                DeckCommand::ExtraLeadUs(us) => self.extra_lead_us = us,
                DeckCommand::Steady(steady) => {
                    self.steady = steady;
                    self.anchor_pos = self.pos;
                    self.anchor_rev = angle_rev_now;
                }
                DeckCommand::Transport(mode) => self.transport = mode,
                DeckCommand::Tempo(t) => self.tempo = t,
                DeckCommand::PlatterRpm(rpm) => {
                    if (rpm - self.platter_rpm).abs() > 1e-9 {
                        self.platter_rpm = rpm;
                        self.update_samples_per_rev();
                        // The scale changed; keep the playhead where it is.
                        self.anchor_pos = self.pos;
                        self.anchor_rev = angle_rev_now;
                    }
                }
            }
        }
    }

    /// Flywheel or hands-on velocity for this block, see the module doc.
    /// FREE and BRAKE command zero speed, so any motion there is a hand
    /// and tracks fast; without a flywheel estimate yet, so does everything.
    fn blend_velocity(&mut self, fast: Option<f64>, slow: Option<f64>, now: u64, block_s: f64) -> f64 {
        let (Some(f), Some(s)) = (fast, slow) else {
            self.hands = 1.0;
            self.hands_since = now;
            return fast.unwrap_or(0.0);
        };
        let nominal = self.platter_rpm / 60.0;
        let commanded = if self.transport == Mode::Spin { nominal * self.tempo } else { 0.0 };
        let dev_slow = (f - s).abs() / nominal;
        let dev_cmd = (f - commanded).abs() / nominal;
        if dev_slow > HANDS_ON_FRAC || dev_cmd > HANDS_CMD_FRAC {
            self.hands = 1.0;
            self.hands_since = now;
        } else if dev_slow > 0.5 * HANDS_ON_FRAC || dev_cmd > 0.5 * HANDS_CMD_FRAC {
            self.hands_since = now;
        } else if now.saturating_sub(self.hands_since) > HANDS_OFF_US {
            self.hands = (self.hands - block_s / HANDS_BLEND_S).max(0.0);
        }
        self.hands * f + (1.0 - self.hands) * s
    }

    /// Fill `out` (interleaved, `channels` wide) at `out_rate` Hz.
    /// `latency_us`: when the first frame reaches the DAC, relative to now.
    pub fn render(&mut self, out: &mut [f32], channels: usize, out_rate: f64, latency_us: u64) {
        self.render_at(out, channels, out_rate, latency_us, now_us());
    }

    /// [`Deck::render`] with an explicit clock, for tests.
    pub fn render_at(&mut self, out: &mut [f32], channels: usize, out_rate: f64, latency_us: u64, now: u64) {
        let channels = channels.max(1);
        let frames = out.len() / channels;
        let block_s = frames as f64 / out_rate;

        // Platter: where is it at the instant this block reaches the DAC?
        let p = self.platter.snapshot();
        let age_us = now.saturating_sub(p.t_us);
        let stale = !p.connected || age_us > STALE_US;
        let lead_us = (latency_us + self.extra_lead_us.max(0.0) as u64).min(MAX_LEAD_US);
        let buffer_us = if p.wifi { WIFI_BUFFER_US } else { 0 };
        let t_start = (now + lead_us).saturating_sub(buffer_us);
        // `ready`: enough history for a velocity estimate. Until then the
        // playhead holds rather than anchoring at a guessed speed.
        let (angle, vel, ready) = if stale {
            self.timeline_valid = false;
            self.hands = 1.0;
            (self.timeline_angle, 0.0, false)
        } else {
            // Interpolate inside the captured history; beyond its newest
            // sample extrapolate with the smoothed velocity, which fades
            // out if the data stops coming.
            let t_data = t_start.min(p.t_us);
            let base = self.platter.angle_at(t_data).unwrap_or(p.angle_rev());
            let fast = self.platter.velocity_at(t_data, VEL_WINDOW_US);
            let slow = self.platter.velocity_at(t_data, FLYWHEEL_WINDOW_US);
            let mut v = self.blend_velocity(fast, slow, now, block_s);
            let gap = t_start - t_data;
            if gap > HOLD_US {
                v *= (-((gap - HOLD_US) as f64) / DECAY_TAU_US).exp();
            }
            (base + v * (gap.min(MAX_EXTRAP_US) as f64 * 1e-6), v, fast.is_some())
        };
        self.drain_commands(angle);

        self.rb.block_frames.store(frames as u32, Ordering::Relaxed);
        self.rb.out_rate.store(out_rate as u32, Ordering::Relaxed);
        // Platter-to-DAC delay: the Wi-Fi playback delay already contains the
        // output latency; it only grows if the latency exceeds it.
        self.rb.lead_us.store(buffer_us.max(lead_us) as u32, Ordering::Relaxed);
        self.rb.callbacks.fetch_add(1, Ordering::Relaxed);

        let Some(track) = self.track.clone() else {
            out.fill(0.0);
            self.motion_gain = 0.0;
            self.rate = 0.0;
            self.rb.pos_bits.store(self.pos.to_bits(), Ordering::Relaxed);
            self.rb.rate_bits.store(0f64.to_bits(), Ordering::Relaxed);
            self.rb.set_peaks(0.0, 0.0);
            return;
        };

        // Nominal is track_rate/out_rate track samples per output frame.
        let nominal = track.sample_rate as f64 / out_rate;
        let feed_forward = vel * self.samples_per_rev / out_rate;
        let stationary = ready && p.wifi && self.platter.stationary();
        let source_changed = self.timeline_wifi != p.wifi;
        self.timeline_wifi = p.wifi;
        if ready {
            if !self.timeline_valid || source_changed {
                // New timeline segment (start, reconnect, source switch):
                // continue from the current playhead, at the platter's speed.
                self.anchor_pos = self.pos;
                self.anchor_rev = angle;
                self.timeline_valid = true;
                if !self.steady {
                    self.rate = if stationary { 0.0 } else { feed_forward };
                }
            }
            self.timeline_angle = angle;
        }

        let mut end_target = None;
        let mut pos_err = 0.0;
        let mut r_end = if self.steady {
            let running = self.transport == Mode::Spin
                && status::mode(p.status) == Mode::Spin
                && p.status & (status::FAULT | status::LINK_TIMEOUT) == 0
                && !stale;
            let r = if running { nominal * self.tempo } else { 0.0 };
            // A deck starts and stops dead, not with a glide.
            self.rate = r;
            r
        } else if !ready || stationary {
            // Absorb rest chatter into the anchor so it cannot accumulate.
            self.anchor_pos = self.pos;
            self.anchor_rev = angle;
            0.0
        } else {
            // Tracking loop: feed-forward velocity plus a slow pull toward
            // the exact platter position at the end of this block.
            let angle_end = angle + vel * block_s;
            let target = self.anchor_pos + (angle_end - self.anchor_rev) * self.samples_per_rev;
            let predicted = self.pos + frames as f64 * (self.rate + feed_forward) * 0.5;
            end_target = Some(target);
            pos_err = target - predicted;
            if pos_err.abs() > POS_ERR_HANDS {
                self.hands = 1.0;
                self.hands_since = now;
            }
            let tau = self.hands * TRACK_TAU_S + (1.0 - self.hands) * FLYWHEEL_TAU_S;
            feed_forward + pos_err / (tau * out_rate)
        };
        // Cap so a glitch can never tear through the buffer. A discontinuity
        // (link glitch or re-anchor race) snaps onto the target instead.
        let cap = RATE_CAP * nominal;
        if r_end.abs() > cap {
            r_end = r_end.signum() * cap;
            self.rate = r_end;
            if let Some(t) = end_target {
                self.pos = t - r_end * frames as f64;
            }
        }
        self.rb.rate_bits.store((r_end / nominal).to_bits(), Ordering::Relaxed);
        if let Some(t) = &self.tap {
            t.send(Event::Block {
                t_us: now,
                pos: self.pos,
                rate: r_end / nominal,
                vel_rev_s: vel,
                pos_err,
                lead_us: buffer_us.max(lead_us) as u32,
                ready,
                stale,
                stationary,
                steady: self.steady,
                hands: self.hands > 0.5,
            });
        }

        // A parked playhead is silence, not a repeated sample (DC). Fade
        // within 3 ms to avoid clicks when hand motion starts or stops.
        let moving = r_end.abs() > nominal * 0.001 || self.rate.abs() > nominal * 0.001;
        let motion_target = if moving { 1.0f32 } else { 0.0 };
        let motion_step = (1.0 / (out_rate * 0.003)) as f32;

        let s = &track.samples;
        let n = track.frames as i64;
        let g_step = (self.gain - self.gain_now) / frames.max(1) as f32;
        let r_step = (r_end - self.rate) / frames.max(1) as f64;
        let (mut peak_l, mut peak_r) = (0f32, 0f32);
        for f in 0..frames {
            self.gain_now += g_step;
            self.rate += r_step;
            self.motion_gain += (motion_target - self.motion_gain).clamp(-motion_step, motion_step);
            let i = self.pos.floor() as i64;
            let t = (self.pos - i as f64) as f32;
            let (l, r) = if i >= 1 && i + 2 < n {
                let b = (i as usize - 1) * 2;
                (
                    catmull(s[b], s[b + 2], s[b + 4], s[b + 6], t),
                    catmull(s[b + 1], s[b + 3], s[b + 5], s[b + 7], t),
                )
            } else if i >= 0 && i < n {
                let b = i as usize * 2;
                (s[b], s[b + 1])
            } else {
                (0.0, 0.0)
            };
            let o = &mut out[f * channels..(f + 1) * channels];
            let gain = self.gain_now * self.motion_gain;
            let (gl, gr) = (l * gain, r * gain);
            peak_l = peak_l.max(gl.abs());
            peak_r = peak_r.max(gr.abs());
            if channels == 1 {
                o[0] = 0.5 * (gl + gr);
            } else {
                o[0] = gl;
                o[1] = gr;
                for x in &mut o[2..] {
                    *x = 0.0;
                }
            }
            self.pos += self.rate;
        }
        self.gain_now = self.gain;
        self.rate = r_end;
        self.rb.pos_bits.store(self.pos.to_bits(), Ordering::Relaxed);
        self.rb.set_peaks(peak_l, peak_r);
    }
}

#[inline]
fn catmull(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
    let a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
    let b = p0 - 2.5 * p1 + 2.0 * p2 - 0.5 * p3;
    let c = -0.5 * p0 + 0.5 * p2;
    ((a * t + b) * t + c) * t + p1
}

#[cfg(test)]
mod tests {
    use super::*;
    use platter_proto::ANGLE_UNITS_PER_REV;

    const SR: u32 = 48_000;
    const BLOCK: usize = 480; // 10 ms, the WASAPI shared-mode period
    const UNITS: f64 = ANGLE_UNITS_PER_REV as f64;
    /// Track samples per platter revolution at 33 1/3 and 48 kHz.
    const SPR: f64 = SR as f64 * SECONDS_PER_REV;

    fn ramp_track(seconds: usize) -> Arc<Track> {
        let frames = SR as usize * seconds;
        Arc::new(Track {
            name: "ramp".into(),
            sample_rate: SR,
            frames,
            samples: (0..frames).flat_map(|i| [i as f32, i as f32]).collect(),
            overview: vec![],
        })
    }

    fn deck_with(track: Arc<Track>) -> (Deck, Arc<PlatterState>, Arc<DeckReadback>) {
        let platter = Arc::new(PlatterState::new());
        let rb = Arc::new(DeckReadback::default());
        let (tx, rx) = crossbeam_channel::unbounded();
        let (ttx, trx) = crossbeam_channel::bounded(4);
        std::mem::forget(trx);
        let deck = Deck::new(rx, ttx, platter.clone(), rb.clone(), None);
        tx.send(DeckCommand::Load(Some(track))).unwrap();
        std::mem::forget(tx);
        platter.set_connected(true);
        (deck, platter, rb)
    }

    /// Publish 2 ms encoder samples of a platter turning at `rev_per_s(t)`
    /// up to and including time `to`, integrating from t = 0.
    struct Spinner {
        next: u64,
        angle: f64,
        /// Encoder noise, uniform within +-units, deterministic.
        noise_units: i64,
        seed: u64,
    }
    impl Spinner {
        fn new() -> Self {
            Spinner { next: 0, angle: 0.0, noise_units: 0, seed: 0x9E37_79B9_7F4A_7C15 }
        }
        fn run(&mut self, platter: &PlatterState, to: u64, rev_per_s: impl Fn(u64) -> f64) {
            while self.next <= to {
                let t = self.next;
                self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let noise = if self.noise_units > 0 {
                    ((self.seed >> 33) % (2 * self.noise_units as u64 + 1)) as i64 - self.noise_units
                } else {
                    0
                };
                platter.publish((self.angle * UNITS).round() as i64 + noise, 0, status::SENSOR_OK, t);
                self.angle += rev_per_s(t) * 0.002;
                self.next += 2000;
            }
        }
    }

    #[test]
    fn catmull_hits_knots_and_is_linear_on_ramps() {
        assert!((catmull(0.0, 1.0, 2.0, 3.0, 0.0) - 1.0).abs() < 1e-6);
        assert!((catmull(0.0, 1.0, 2.0, 3.0, 1.0) - 2.0).abs() < 1e-6);
        assert!((catmull(0.0, 1.0, 2.0, 3.0, 0.25) - 1.25).abs() < 1e-6);
    }

    #[test]
    fn steady_playback_ignores_motor_ripple_but_obeys_pitch_stop_and_disconnect() {
        let (mut deck, platter, rb) = deck_with(ramp_track(8));
        platter.set_wifi(true);
        deck.steady = true;
        deck.transport = Mode::Spin;
        let mut buf = vec![0.0; 2 * BLOCK];
        let start = 1_000_000;
        for block in 0..150u64 {
            let now = start + block * 10_000;
            // More ripple than the live trace: wildly alternating speed,
            // including backwards motion, cannot alter steady audio tempo.
            let angle = (block as f64 * 364.09 + (block as f64 * 0.49).sin() * 300.0) as i64;
            platter.publish(angle, if block % 2 == 0 { 1500 } else { 5500 },
                status::with_mode(status::SENSOR_OK | status::ENABLED, Mode::Spin), now);
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
            assert!((rb.rate() - 1.0).abs() < 1e-9);
        }
        assert!((rb.pos() - 150.0 * BLOCK as f64).abs() < 1e-6);
        deck.tempo = 1.08;
        deck.render_at(&mut buf, 2, SR as f64, 0, start + 1_500_000);
        assert!((rb.rate() - 1.08).abs() < 1e-9);
        deck.transport = Mode::Brake;
        deck.render_at(&mut buf, 2, SR as f64, 0, start + 1_510_000);
        let stopped = rb.pos();
        assert_eq!(rb.rate(), 0.0);
        assert!(buf[300..].iter().all(|&v| v == 0.0));
        deck.transport = Mode::Spin;
        platter.set_connected(false);
        deck.render_at(&mut buf, 2, SR as f64, 0, start + 1_520_000);
        assert_eq!(rb.pos(), stopped);
        assert!(buf.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn platter_speed_sets_scratch_scale_not_tempo() {
        // STEADY at 45: still 1.00x at pitch 0.
        let (mut deck, platter, rb) = deck_with(ramp_track(4));
        deck.steady = true;
        deck.transport = Mode::Spin;
        let mut buf = vec![0.0; 2 * BLOCK];
        platter.publish(0, 0, status::with_mode(status::SENSOR_OK | status::ENABLED, Mode::Spin), 1_000_000);
        deck.render_at(&mut buf, 2, SR as f64, 0, 1_000_000);
        deck.platter_rpm = 45.0;
        deck.update_samples_per_rev();
        deck.render_at(&mut buf, 2, SR as f64, 0, 1_010_000);
        assert!((rb.rate() - 1.0).abs() < 1e-9);
        // VINYL at 45 with the platter actually turning at 45 rpm: 1.00x.
        let (mut deck, platter, rb) = deck_with(ramp_track(6));
        let mut spinner = Spinner::new();
        for block in 0..100u64 {
            let now = block * 10_000;
            spinner.run(&platter, now, |_| 0.75);
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
            if block == 0 {
                deck.platter_rpm = 45.0;
                deck.update_samples_per_rev();
            }
        }
        assert!((rb.rate() - 1.0).abs() < 0.005, "rate {}", rb.rate());
    }

    #[test]
    fn stationary_encoder_noise_is_silent_and_hand_motion_still_scratches() {
        let (mut deck, platter, rb) = deck_with(ramp_track(4));
        platter.set_wifi(true);
        let mut buf = vec![0.0; 2 * BLOCK];
        let start = 1_000_000;
        platter.publish(65536, 0, status::SENSOR_OK, start);
        deck.render_at(&mut buf, 2, SR as f64, 0, start);
        // Park on nonzero audio: holding a sample would produce DC/wind.
        deck.pos = 24000.0;
        deck.anchor_pos = deck.pos;
        for tick in 1..=250u64 {
            let now = start + tick * 2000;
            platter.publish(65536 + [0, 8, -8, 4, -4][tick as usize % 5],
                100, status::SENSOR_OK, now);
            if tick % 5 == 0 { deck.render_at(&mut buf, 2, SR as f64, 0, now); }
            if tick >= 30 {
                assert_eq!(rb.rate(), 0.0);
                assert!(buf.iter().all(|&v| v == 0.0), "rest noise is audible");
            }
        }
        let stopped = rb.pos();
        // Hand motion at ~0.5 rev/s. It reaches the audio after the 40 ms
        // Wi-Fi playback delay, then the playhead moves with it.
        for tick in 251..=300u64 {
            let now = start + tick * 2000;
            platter.publish(65536 + (tick as i64 - 250) * 64, 3000, status::SENSOR_OK, now);
            if tick % 5 == 0 { deck.render_at(&mut buf, 2, SR as f64, 0, now); }
        }
        assert!(rb.pos() > stopped + 1000.0, "moved {}", rb.pos() - stopped);
        assert!(buf.iter().any(|&v| v != 0.0));
        for tick in 301..=350u64 {
            let now = start + tick * 2000;
            platter.publish(65536 + (350 - tick as i64) * 64, -3000, status::SENSOR_OK, now);
            if tick % 5 == 0 { deck.render_at(&mut buf, 2, SR as f64, 0, now); }
        }
        assert!(rb.rate() < -0.5, "reverse scratch suppressed: {}", rb.rate());
    }

    #[test]
    fn eject_clears_position_rate_and_audio() {
        let (mut deck, platter, rb) = deck_with(ramp_track(2));
        let (tx, rx) = crossbeam_channel::unbounded();
        let mut buf = vec![0.0; 2 * BLOCK];
        platter.publish(0, 0, 0, 1_000_000);
        deck.render_at(&mut buf, 2, SR as f64, 0, 1_000_000);
        deck.rx = rx;
        tx.send(DeckCommand::Jump(24000.0)).unwrap();
        deck.render_at(&mut buf, 2, SR as f64, 0, 1_010_000);
        assert_eq!(rb.pos(), 24000.0);
        tx.send(DeckCommand::Load(None)).unwrap();
        deck.render_at(&mut buf, 2, SR as f64, 0, 1_020_000);
        assert_eq!(rb.pos(), 0.0);
        assert_eq!(rb.rate(), 0.0);
        assert_eq!(rb.peaks(), (0.0, 0.0));
        assert!(buf.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn wifi_jitter_does_not_modulate_constant_speed_audio() {
        let (mut deck, platter, rb) = deck_with(ramp_track(4));
        platter.set_wifi(true);
        let mut buf = vec![0f32; 2 * BLOCK];
        let start = 1_000_000u64;
        let mut next = 0u64;
        let mut worst: f64 = 0.0;
        for block in 0..150u64 {
            let now = start + block * 10000;
            // Bursty delivery with missing packets; capture timestamps remain
            // accurate. Deliberately noisy instantaneous velocity must not
            // modulate playback when timestamp interpolation is available.
            while start + next * 2000 + [1000, 7000, 3000, 9000][next as usize % 4] <= now {
                if next % 7 != 0 {
                    let units = ((next * 2000) as f64 / 1.8e6 * 65536.0).round() as i64;
                    platter.publish(units, if next % 2 == 0 { 1500 } else { 5500 }, 2, start + next * 2000);
                }
                next += 1;
            }
            deck.render_at(&mut buf, 2, SR as f64, 20_000, now);
            if block > 5 { worst = worst.max((rb.rate() - 1.0).abs()); }
        }
        assert!(worst < 0.01, "packet jitter changed audio rate by {worst}");
        let pos = rb.pos();
        platter.set_connected(false);
        // A lost link ramps to a stop inside one block, then holds.
        deck.render_at(&mut buf, 2, SR as f64, 20_000, start + 1_510_000);
        assert!((rb.pos() - pos).abs() < BLOCK as f64, "disconnect moved playhead by {}", rb.pos() - pos);
        assert_eq!(rb.rate(), 0.0);
        let held = rb.pos();
        deck.render_at(&mut buf, 2, SR as f64, 20_000, start + 1_520_000);
        assert_eq!(rb.pos(), held);
    }

    #[test]
    fn playhead_tracks_platter_both_directions_without_drift() {
        let (mut deck, platter, rb) = deck_with(ramp_track(8));
        let mut buf = vec![0f32; 2 * BLOCK];
        let mut spinner = Spinner::new();
        // Forward at 33 1/3 for 1.5 s: the playhead sits within 1 ms of
        // audio of the platter position and the rate within 0.5 %. The
        // timeline anchors at block 1, the first with a velocity estimate.
        for block in 0..150u64 {
            let now = block * 10_000;
            spinner.run(&platter, now, |_| 1.0 / SECONDS_PER_REV);
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
            if block >= 50 {
                let expect = block as f64 * BLOCK as f64;
                assert!((rb.pos() - expect).abs() < 48.0, "block {block}: pos {} expect {expect}", rb.pos());
                assert!((rb.rate() - 1.0).abs() < 0.005, "block {block}: rate {}", rb.rate());
            }
        }
        let turn = rb.pos();
        // Reverse at half speed: rate settles at -0.5 and the playhead
        // comes back by the right amount.
        for block in 150..300u64 {
            let now = block * 10_000;
            spinner.run(&platter, now, |t| if t >= 1_500_000 { -0.5 / SECONDS_PER_REV } else { 1.0 / SECONDS_PER_REV });
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
        }
        assert!((rb.rate() + 0.5).abs() < 0.005, "rate {}", rb.rate());
        let expect = turn - 1.5 * 0.5 * SR as f64;
        assert!((rb.pos() - expect).abs() < 48.0, "pos {} expect {expect}", rb.pos());
    }

    #[test]
    fn output_latency_leads_the_platter() {
        // The block rendered now reaches the DAC `latency` later, so the
        // deck reads the platter timeline that much ahead. The platter
        // stops dead at 1.0 s; with 20 ms of latency the audio stops two
        // blocks earlier than with none. (The test feeds samples 20 ms
        // ahead of the clock so the timeline is known that far.)
        let mut stop_block = [None, None];
        for (i, latency) in [0u64, 20_000].into_iter().enumerate() {
            let (mut deck, platter, rb) = deck_with(ramp_track(8));
            let mut buf = vec![0f32; 2 * BLOCK];
            let mut spinner = Spinner::new();
            for block in 0..150u64 {
                let now = block * 10_000;
                spinner.run(&platter, now + 20_000, |t| if t < 1_000_000 { 1.0 / SECONDS_PER_REV } else { 0.0 });
                deck.render_at(&mut buf, 2, SR as f64, latency, now);
                if latency > 0 {
                    assert_eq!(rb.lead_us.load(Ordering::Relaxed), latency as u32);
                }
                if stop_block[i].is_none() && block > 10 && rb.rate() < 0.5 {
                    stop_block[i] = Some(block);
                }
            }
        }
        let (plain, led) = (stop_block[0].unwrap(), stop_block[1].unwrap());
        assert_eq!(led + 2, plain, "latency 0 stops at block {plain}, 20 ms at {led}");
    }

    #[test]
    fn rate_is_continuous_through_motor_ripple() {
        // 15 % speed ripple at the 46 Hz cogging frequency (84 cycles/rev at
        // 33 rpm). Raw per-block rates would step by up to 30 % every 10 ms;
        // the tracking loop ramps smoothly and averages most of it out.
        let (mut deck, platter, rb) = deck_with(ramp_track(8));
        let mut buf = vec![0f32; 2 * BLOCK];
        let mut spinner = Spinner::new();
        let mut worst_step = 0f32;
        let mut worst_rate = 0f64;
        let mut last_out: Option<f32> = None;
        for block in 0..200u64 {
            let now = block * 10_000;
            spinner.run(&platter, now, |t| {
                (1.0 + 0.15 * (t as f64 * 1e-6 * 46.0 * std::f64::consts::TAU).sin()) / SECONDS_PER_REV
            });
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
            if block >= 20 {
                // The ramp track reads back the playhead: consecutive output
                // differences are the per-frame rate.
                for f in 0..BLOCK {
                    let v = buf[f * 2];
                    if let Some(prev) = last_out {
                        let step = v - prev;
                        worst_step = worst_step.max((step - 1.0).abs());
                    }
                    last_out = Some(v);
                }
                worst_rate = worst_rate.max((rb.rate() - 1.0).abs());
            }
        }
        assert!(worst_step < 0.05, "per-frame rate deviates by {worst_step}");
        assert!(worst_rate < 0.05, "block rate deviates by {worst_rate}");
    }

    #[test]
    fn flywheel_averages_ripple_and_encoder_noise_while_spinning_free() {
        // Residual ripple as measured after cogging compensation (1 % at
        // 14x, 2 % at 48x, 1.5 % at 84x of 33 rpm) plus +-8 units (2 counts)
        // of encoder noise. Fast tracking alone passes about 2 % of it to
        // the pitch; the flywheel passes under 0.4 %, with the playhead
        // still on the platter position.
        let (mut deck, platter, rb) = deck_with(ramp_track(8));
        deck.transport = Mode::Spin;
        let mut buf = vec![0f32; 2 * BLOCK];
        let mut spinner = Spinner::new();
        spinner.noise_units = 8;
        let speed = |t: u64| {
            let s = t as f64 * 1e-6 * std::f64::consts::TAU;
            (1.0 + 0.01 * (s * 7.8).sin() + 0.02 * (s * 26.7).sin() + 0.015 * (s * 46.7).sin()) / SECONDS_PER_REV
        };
        let mut a0 = 0.0;
        let mut worst = 0f64;
        let mut worst_pos = 0f64;
        for block in 0..400u64 {
            let now = block * 10_000;
            spinner.run(&platter, now, speed);
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
            if block == 1 {
                a0 = platter.angle_at(now).unwrap();
            }
            if block >= 100 {
                worst = worst.max((rb.rate() - 1.0).abs());
                // The playhead after a block is the platter position at the
                // end of that block, one nominal block ahead of `now`.
                let expect = (platter.angle_at(now).unwrap() - a0) * SPR + BLOCK as f64;
                worst_pos = worst_pos.max((rb.pos() - expect).abs());
            }
        }
        assert!(deck.hands < 0.01, "flywheel not engaged: hands {}", deck.hands);
        assert!(worst < 0.004, "flywheel rate deviates by {worst}");
        assert!(worst_pos < 200.0, "playhead off the platter by {worst_pos} samples");
    }

    #[test]
    fn hand_on_the_platter_switches_to_fast_tracking_and_back() {
        // Free spin with 2 % ripple for 1.5 s; a grab brings the platter to
        // a -0.3x backspin within 100 ms, held 0.5 s; released at 2.1 s it
        // is back at speed by 2.4 s. The grab is tracked within 60 ms, the
        // backspin follows the fast estimate, and the flywheel is back
        // for the free spin afterwards.
        let (mut deck, platter, rb) = deck_with(ramp_track(12));
        deck.transport = Mode::Spin;
        let mut buf = vec![0f32; 2 * BLOCK];
        let mut spinner = Spinner::new();
        spinner.noise_units = 8;
        let speed = |t: u64| {
            let s = t as f64 * 1e-6;
            let base = if s < 1.5 {
                1.0
            } else if s < 1.6 {
                1.0 - 13.0 * (s - 1.5)
            } else if s < 2.1 {
                -0.3
            } else if s < 2.4 {
                -0.3 + (1.3 / 0.3) * (s - 2.1)
            } else {
                1.0
            };
            (base + 0.02 * (s * std::f64::consts::TAU * 26.7).sin()) / SECONDS_PER_REV
        };
        let mut a0 = 0.0;
        let mut worst_free = 0f64;
        let mut worst_back = 0f64;
        let mut worst_settled = 0f64;
        let mut rate_at_grab = 1.0;
        let mut hands_free = 1.0;
        for block in 0..400u64 {
            let now = block * 10_000;
            spinner.run(&platter, now, speed);
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
            match block {
                1 => a0 = platter.angle_at(now).unwrap(),
                100..=149 => {
                    worst_free = worst_free.max((rb.rate() - 1.0).abs());
                    hands_free = deck.hands;
                }
                156 => rate_at_grab = rb.rate(),
                180..=209 => worst_back = worst_back.max((rb.rate() + 0.3).abs()),
                330..=399 => worst_settled = worst_settled.max((rb.rate() - 1.0).abs()),
                _ => {}
            }
        }
        assert!(hands_free < 0.01, "flywheel not engaged before the grab: {hands_free}");
        assert!(worst_free < 0.004, "free spin rate deviates by {worst_free}");
        // 60 ms into the grab the platter is at 0.22x; the fast window
        // already reads well below the flywheel's 1.0x.
        assert!(rate_at_grab < 0.6, "grab not tracked: rate {rate_at_grab} 60 ms in");
        assert!(worst_back < 0.04, "backspin rate deviates by {worst_back}");
        assert!(deck.hands < 0.01, "flywheel not back after release: {}", deck.hands);
        assert!(worst_settled < 0.004, "after release the rate deviates by {worst_settled}");
        let expect = (platter.angle_at(3_990_000).unwrap() - a0) * SPR + BLOCK as f64;
        assert!((rb.pos() - expect).abs() < 200.0, "playhead off by {}", rb.pos() - expect);
    }

    #[test]
    fn stale_platter_holds() {
        let (mut deck, platter, rb) = deck_with(ramp_track(1));
        let mut buf = vec![0f32; 2 * BLOCK];
        let t0 = 1_000_000u64;
        platter.publish(0, 0, 0, t0);
        deck.render_at(&mut buf, 2, SR as f64, 0, t0);
        // Spinning fast, but the sample is a second old: no extrapolation.
        platter.publish(0, 100_000, 0, t0);
        deck.render_at(&mut buf, 2, SR as f64, 0, t0 + 1_000_000);
        assert!(rb.pos().abs() < 1e-9, "pos {}", rb.pos());
        // Link down: same.
        platter.set_connected(false);
        platter.publish(0, 100_000, 0, t0 + 1_000_000);
        deck.render_at(&mut buf, 2, SR as f64, 0, t0 + 1_001_000);
        assert!(rb.pos().abs() < 1e-9, "pos {}", rb.pos());
    }

    #[test]
    fn teleport_snaps_at_rate_cap_and_settles_exactly() {
        // The platter rests, jumps four revolutions between two samples
        // (an impossible 2000x), and rests again. The rate is capped, the
        // playhead snaps rather than racing, and it settles on the new
        // position.
        let (mut deck, platter, rb) = deck_with(ramp_track(12));
        let mut buf = vec![0f32; 2 * BLOCK];
        let mut next = 0u64;
        let mut peak = 0f64;
        for block in 0..200u64 {
            let now = block * 10_000;
            while next <= now {
                let angle = if next < 500_000 { 0 } else { 4 * ANGLE_UNITS_PER_REV };
                platter.publish(angle, 0, status::SENSOR_OK, next);
                next += 2000;
            }
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
            assert!(rb.rate().abs() <= RATE_CAP + 1e-9, "block {block}: rate {}", rb.rate());
            peak = peak.max(rb.rate());
        }
        assert!((peak - RATE_CAP).abs() < 1e-9, "never hit the cap: {peak}");
        // Within 1 ms of audio: the last few samples close at the flywheel's
        // slower time constant once the error is small.
        assert!((rb.pos() - 4.0 * SPR).abs() < 48.0, "pos {} expect {}", rb.pos(), 4.0 * SPR);
        assert!(rb.rate().abs() < 1e-3, "rate {}", rb.rate());
    }

    #[test]
    fn out_of_range_is_silent() {
        // Load anchors at the current angle, so pos is 0. Spinning
        // backwards from there reads before the track start: silence.
        let (mut deck, platter, rb) = deck_with(ramp_track(1));
        let mut buf = vec![1f32; 2 * BLOCK];
        let mut spinner = Spinner::new();
        for block in 0..30u64 {
            let now = block * 10_000;
            spinner.run(&platter, now, |_| -2.0 / SECONDS_PER_REV);
            buf.fill(1.0);
            deck.render_at(&mut buf, 2, SR as f64, 0, now);
        }
        assert!(rb.pos() < -1000.0, "pos {}", rb.pos());
        assert!(buf.iter().all(|&x| x == 0.0));
    }
}
