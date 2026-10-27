//! The one command the PC wants the platter to obey right now. The link
//! (or the sim) reads it every heartbeat and immediately after a change.
use crossbeam_channel::{bounded, Receiver, Sender};
use platter_proto::{HostFrame, Mode};
use std::sync::Mutex;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HostCommand {
    pub mode: Mode,
    /// Torque cap in volts (SimpleFOC `voltage_limit`).
    pub torque_v: f32,
    /// SPIN target, milli-rad/s.
    pub target_vel_mrad: i32,
    pub drag: u8,
}

impl Default for HostCommand {
    fn default() -> Self {
        // 1 V: about 0.15 A through the GM2804's 5.6 ohm phases, enough
        // torque headroom above its ~0.2 V back-EMF at 33 rpm to hold speed
        // against cogging. The board's saved voltage limit still caps it.
        HostCommand { mode: Mode::Free, torque_v: 1.0, target_vel_mrad: platter_proto::RPM33_MRAD, drag: 0 }
    }
}

impl HostCommand {
    pub fn to_frame(&self, seq: u8) -> HostFrame {
        HostFrame {
            seq,
            mode: self.mode,
            torque: HostFrame::torque_from_volts(self.torque_v),
            target_vel_mrad: self.target_vel_mrad,
            drag: self.drag,
        }
    }
}

pub struct CommandSlot {
    cmd: Mutex<HostCommand>,
    notify_tx: Sender<()>,
    notify_rx: Receiver<()>,
}

impl Default for CommandSlot {
    fn default() -> Self {
        Self::new()
    }
}

impl CommandSlot {
    pub fn new() -> Self {
        let (notify_tx, notify_rx) = bounded(1);
        CommandSlot { cmd: Mutex::new(HostCommand::default()), notify_tx, notify_rx }
    }

    pub fn set(&self, cmd: HostCommand) {
        let changed = {
            let mut g = self.cmd.lock().unwrap();
            let changed = *g != cmd;
            *g = cmd;
            changed
        };
        if changed {
            let _ = self.notify_tx.try_send(());
        }
    }

    pub fn get(&self) -> HostCommand {
        *self.cmd.lock().unwrap()
    }

    /// Block until a change is signalled or `timeout` passes. Returns true on change.
    pub fn wait(&self, timeout: Duration) -> bool {
        self.notify_rx.recv_timeout(timeout).is_ok()
    }

    /// Non-blocking: true if a change was signalled since the last check.
    pub fn take_dirty(&self) -> bool {
        self.notify_rx.try_recv().is_ok()
    }
}
