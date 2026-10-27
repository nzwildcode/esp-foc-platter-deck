//! Simulated platter: a software motor driven by the mouse, so the deck
//! runs without the board. Same units, same [`PlatterState`], same
//! [`CommandSlot`] as the Wi-Fi link.
use crate::clock::now_us;
use crate::command::CommandSlot;
use crate::platter::PlatterState;
use platter_proto::{status, Mode, ANGLE_UNITS_PER_REV};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandEvent {
    Grab,
    Release,
}

/// Mouse on the drawn disc. `finger_rev` is unwrapped by the UI.
#[derive(Default, Debug)]
pub struct Hand {
    pub grabbed: bool,
    pub finger_rev: f64,
    pub event: Option<HandEvent>,
}

impl Hand {
    pub fn grab(&mut self, finger_rev: f64) {
        self.grabbed = true;
        self.finger_rev = finger_rev;
        self.event = Some(HandEvent::Grab);
    }
    pub fn drag(&mut self, finger_rev: f64) {
        self.finger_rev = finger_rev;
    }
    pub fn release(&mut self) {
        self.grabbed = false;
        self.event = Some(HandEvent::Release);
    }
}

pub struct SimPlatter {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

/// Angular acceleration per volt of torque cap. A GM2804 on 1 V takes a
/// bare platter from rest to 33 rpm in roughly a quarter second.
const ACCEL_PER_VOLT: f64 = 15.0;
/// Free-wheel coast time constant with drag = 0.
const COAST_TAU_S: f64 = 4.0;
/// Hand velocity low-pass.
const HAND_TAU_S: f64 = 0.015;

impl SimPlatter {
    pub fn spawn(state: Arc<PlatterState>, cmds: Arc<CommandSlot>, hand: Arc<Mutex<Hand>>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let stop = stop.clone();
            std::thread::Builder::new()
                .name("platter-sim".into())
                .spawn(move || run(state, cmds, hand, stop))
                .expect("spawn sim thread")
        };
        SimPlatter { stop, thread: Some(thread) }
    }
}

impl Drop for SimPlatter {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn run(state: Arc<PlatterState>, cmds: Arc<CommandSlot>, hand: Arc<Mutex<Hand>>, stop: Arc<AtomicBool>) {
    // Continue from the current platter angle: switching from a real link
    // to the sim must not move the playhead.
    let mut angle_rev = state.snapshot().angle_rev();
    let mut vel = 0.0f64; // rad/s
    let mut hand_vel = 0.0f64;
    let mut grab_offset = 0.0f64;
    let mut last = Instant::now();
    state.set_connected(true);

    while !stop.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(1));
        let now = Instant::now();
        // Integrate with the measured step: Windows sleep granularity varies.
        let dt = (now - last).as_secs_f64().clamp(1e-4, 0.05);
        last = now;

        let cmd = cmds.get();
        let (grabbed, finger, event) = {
            let mut h = hand.lock().unwrap();
            (h.grabbed, h.finger_rev, h.event.take())
        };

        if event == Some(HandEvent::Grab) {
            grab_offset = angle_rev - finger;
            hand_vel = vel;
        }

        if grabbed {
            let target = finger + grab_offset;
            let inst = (target - angle_rev) * std::f64::consts::TAU / dt;
            let a = (dt / HAND_TAU_S).min(1.0);
            hand_vel += (inst - hand_vel) * a;
            angle_rev = target;
            vel = hand_vel;
        } else {
            if event == Some(HandEvent::Release) {
                vel = hand_vel;
            }
            let accel = (ACCEL_PER_VOLT * cmd.torque_v as f64).max(1.0);
            let step = accel * dt;
            match cmd.mode {
                Mode::Free | Mode::Estop => {
                    let tau = COAST_TAU_S / (1.0 + cmd.drag as f64 / 32.0);
                    vel -= vel * (dt / tau).min(1.0);
                }
                Mode::Spin => {
                    let target = cmd.target_vel_mrad as f64 * 1e-3;
                    vel += (target - vel).clamp(-step, step);
                }
                Mode::Brake => {
                    vel += (0.0 - vel).clamp(-step, step);
                }
            }
            angle_rev += vel * dt / std::f64::consts::TAU;
        }

        let base = if cmd.mode == Mode::Estop { status::SENSOR_OK } else { status::ENABLED | status::SENSOR_OK };
        state.publish(
            (angle_rev * ANGLE_UNITS_PER_REV as f64).round() as i64,
            (vel * 1e3).round() as i32,
            status::with_mode(base, cmd.mode),
            now_us(),
        );
    }
    state.set_connected(false);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_continues_from_current_angle() {
        let state = Arc::new(PlatterState::new());
        let cmds = Arc::new(CommandSlot::new());
        let hand = Arc::new(Mutex::new(Hand::default()));
        // A previous source left the platter at 5 revs, stopped, FREE.
        state.publish(5 * ANGLE_UNITS_PER_REV, 0, 0, now_us());
        let sim = SimPlatter::spawn(state.clone(), cmds, hand);
        std::thread::sleep(Duration::from_millis(80));
        let a = state.snapshot().angle_rev();
        drop(sim);
        assert!((a - 5.0).abs() < 1e-6, "sim started at {a} rev, expected 5");
    }
}
