//! UDP socket worker. Only fresh, ordered samples reach the audio engine.
use crate::tap::{Event, Tap};
use crate::{clock::now_us, CommandSlot, HostCommand, PlatterState};
use anyhow::{Context, Result};
use platter_proto::{status, wifi::*, Mode, PlatterFrame};
use std::net::UdpSocket;
use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    Arc,
};
use std::thread::JoinHandle;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub struct WifiLink {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}
impl WifiLink {
    pub fn open(
        address: &str,
        state: Arc<PlatterState>,
        cmds: Arc<CommandSlot>,
        tap: Option<Arc<Tap>>,
    ) -> Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0").context("bind platter Wi-Fi socket")?;
        socket
            .connect(address)
            .with_context(|| format!("connect UDP to {address}"))?;
        socket.set_read_timeout(Some(Duration::from_millis(2)))?;
        socket.set_write_timeout(Some(Duration::from_millis(5)))?;
        force_free(&cmds);
        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        let thread = std::thread::Builder::new()
            .name("platter-wifi".into())
            .spawn(move || run(socket, state, cmds, flag, tap))?;
        Ok(Self {
            stop,
            thread: Some(thread),
        })
    }
}
impl Drop for WifiLink {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
fn force_free(cmds: &CommandSlot) {
    cmds.set(HostCommand {
        mode: Mode::Free,
        ..cmds.get()
    });
}
fn session_id() -> u32 {
    static NEXT: AtomicU32 = AtomicU32::new(1);
    let t = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    ((t as u32) ^ ((t >> 32) as u32) ^ NEXT.fetch_add(1, Ordering::Relaxed)).max(1)
}

/// Board clock drift the offset tracker tolerates, ppm. The ESP32 crystal and
/// the PC clock differ by tens of ppm. A pure minimum offset never rises when
/// the board clock is the slow one, so the apparent delay would grow about
/// 3 ms per minute until playback went stale and every sample was rejected.
/// Letting the floor rise at this rate keeps the estimate honest either way.
const DRIFT_PPM: i128 = 200;

// Maps the board's wrapping clock to the host using the minimum observed
// transit offset, allowed to drift upward at DRIFT_PPM. This measures excess
// delay/jitter, not absolute one-way latency.
#[derive(Default)]
struct Receiver {
    boot: Option<u32>,
    last_seq: Option<u32>,
    last_time: u32,
    board_us: u64,
    /// Smallest `host - board` seen, and the board time it was seen at; the
    /// usable floor rises from there at DRIFT_PPM.
    offset: i128,
    offset_board_us: u64,
    last_angle: Option<i32>,
    acc: i64,
    dropped: u32,
    rejected: u32,
    delay_us: u32,
    max_gap_us: u32,
    last_arrival: u64,
}
impl Receiver {
    fn accept(&mut self, p: Packet, now: u64) -> Option<(PlatterFrame, u64)> {
        if p.kind != TELEMETRY || p.boot == 0 {
            return None;
        }
        if self.boot.is_some_and(|b| b != p.boot) {
            return None;
        }
        let frame = PlatterFrame::decode(&p.payload).ok()?;
        if let Some(seq) = self.last_seq {
            if !newer(p.seq, seq) || !newer(p.time_us, self.last_time) {
                self.rejected = self.rejected.saturating_add(1);
                return None;
            }
            let board_us = self.board_us + p.time_us.wrapping_sub(self.last_time) as u64;
            let since_min = board_us.saturating_sub(self.offset_board_us) as i128;
            let floor = self.offset + since_min * DRIFT_PPM / 1_000_000;
            let candidate = now as i128 - board_us as i128;
            let offset = if candidate < floor {
                self.offset = candidate;
                self.offset_board_us = board_us;
                candidate
            } else {
                floor
            };
            let estimated = (board_us as i128 + offset).max(0) as u64;
            self.delay_us = now.saturating_sub(estimated).min(u32::MAX as u64) as u32;
            if self.delay_us as u64 > TIMEOUT_US {
                self.rejected = self.rejected.saturating_add(1);
                return None;
            }
            self.dropped = self
                .dropped
                .saturating_add(p.seq.wrapping_sub(seq).saturating_sub(1));
            self.board_us = board_us;
        } else {
            self.boot = Some(p.boot);
            self.board_us = p.time_us as u64;
            self.offset = now as i128 - self.board_us as i128;
            self.offset_board_us = self.board_us;
        }
        if self.last_arrival != 0 {
            self.max_gap_us = self
                .max_gap_us
                .max(now.saturating_sub(self.last_arrival).min(u32::MAX as u64) as u32);
        }
        self.last_arrival = now;
        self.last_seq = Some(p.seq);
        self.last_time = p.time_us;
        if let Some(prev) = self.last_angle {
            self.acc += frame.angle.wrapping_sub(prev) as i64;
        }
        self.last_angle = Some(frame.angle);
        Some((frame, now.saturating_sub(self.delay_us as u64)))
    }
}

fn run(
    socket: UdpSocket,
    state: Arc<PlatterState>,
    cmds: Arc<CommandSlot>,
    stop: Arc<AtomicBool>,
    tap: Option<Arc<Tap>>,
) {
    let mut session = session_id();
    let mut seq = 0u32;
    let mut rx = Receiver {
        acc: state.snapshot().angle_units,
        ..Default::default()
    };
    let mut last_tx = 0;
    let mut invalid = 0u32;
    let mut ready = false;
    let mut buf = [0u8; 2048];
    while !stop.load(Ordering::Relaxed) {
        let now = now_us();
        if rx.boot.is_some() && now.saturating_sub(rx.last_arrival) > TIMEOUT_US {
            state.set_connected(false);
            ready = false;
            force_free(&cmds);
            session = session_id();
            rx = Receiver {
                acc: rx.acc,
                dropped: rx.dropped,
                rejected: rx.rejected,
                max_gap_us: rx.max_gap_us,
                ..Default::default()
            };
            last_tx = 0;
        }
        if !ready {
            force_free(&cmds);
        }
        if last_tx == 0 || cmds.take_dirty() || now.saturating_sub(last_tx) >= HEARTBEAT_MS * 1000 {
            let p = Packet {
                kind: COMMAND,
                session,
                seq,
                time_us: rx.last_time,
                boot: rx.boot.unwrap_or(0),
                payload: cmds.get().to_frame(seq as u8).encode(),
            };
            let _ = socket.send(&p.encode());
            seq = seq.wrapping_add(1);
            last_tx = now;
        }
        match socket.recv(&mut buf) {
            Ok(n) => {
                if n == 0 || n > PACKET_LEN * 4 || n % PACKET_LEN != 0 {
                    invalid = invalid.saturating_add(1);
                } else {
                    for raw in buf[..n].chunks_exact(PACKET_LEN) {
                        if let Some(p) = Packet::decode(raw) {
                            if p.session != session {
                                continue;
                            }
                            if let Some((f, sample_time)) = rx.accept(p, now_us()) {
                                if f.status & status::LINK_TIMEOUT != 0 {
                                    force_free(&cmds);
                                    ready = false;
                                } else if status::mode(f.status) == Mode::Free {
                                    ready = true;
                                }
                                if status::mode(f.status) == Mode::Estop
                                    && matches!(cmds.get().mode, Mode::Spin | Mode::Brake)
                                {
                                    cmds.set(HostCommand {
                                        mode: Mode::Estop,
                                        ..cmds.get()
                                    });
                                }
                                state.publish(rx.acc, f.velocity_mrad, f.status, sample_time);
                                state.set_connected(ready);
                                if let Some(t) = &tap {
                                    t.send(Event::Sample {
                                        t_us: sample_time,
                                        angle: rx.acc,
                                        vel_mrad: f.velocity_mrad,
                                        status: f.status,
                                        delay_us: rx.delay_us,
                                        abs_angle: f.angle,
                                    });
                                }
                            }
                        } else {
                            invalid = invalid.saturating_add(1);
                        }
                    }
                }
                state.set_stats(invalid, rx.dropped);
                state.set_wifi_stats(rx.rejected, rx.delay_us, rx.max_gap_us);
            }
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) => {}
            Err(_) => std::thread::sleep(Duration::from_millis(5)),
        }
    }
    let p = Packet {
        kind: COMMAND,
        session,
        seq,
        time_us: rx.last_time,
        boot: rx.boot.unwrap_or(0),
        payload: HostCommand::default().to_frame(seq as u8).encode(),
    };
    let _ = socket.send(&p.encode());
    state.set_connected(false);
}

#[cfg(test)]
mod tests {
    use super::*;
    fn packet(seq: u32, time_us: u32, angle: i32) -> Packet {
        Packet {
            kind: TELEMETRY,
            session: 1,
            seq,
            time_us,
            boot: 99,
            payload: PlatterFrame {
                seq: seq as u8,
                angle,
                velocity_mrad: 3491,
                status: 2,
            }
            .encode(),
        }
    }
    #[test]
    fn duplicates_reordering_loss_and_clock_wrap() {
        let mut r = Receiver::default();
        assert!(r
            .accept(packet(u32::MAX, u32::MAX - 999, i32::MAX - 5), 10000)
            .is_some());
        assert!(r.accept(packet(0, 1000, i32::MIN + 4), 12000).is_some());
        assert_eq!(r.acc, 10);
        assert!(r.accept(packet(0, 1000, 0), 13000).is_none());
        assert!(r.accept(packet(u32::MAX, 0, 0), 13000).is_none());
        assert!(r.accept(packet(3, 7000, i32::MIN + 20), 18000).is_some());
        assert_eq!(r.dropped, 2);
        assert_eq!(r.rejected, 2);
        assert_eq!(r.acc, 26);
    }
    #[test]
    fn delayed_data_and_other_boot_do_not_move_playhead() {
        let mut r = Receiver::default();
        r.accept(packet(1, 1000, 50), 10000).unwrap();
        // 2 ms of board time, but arriving more than the timeout after the
        // expected instant: stale, rejected.
        assert!(r.accept(packet(2, 3000, 5000), 12_000 + TIMEOUT_US + 1).is_none());
        let mut other = packet(3, 5000, 5000);
        other.boot = 100;
        assert!(r.accept(other, 14000).is_none());
        assert_eq!(r.acc, 0);
        assert!(r.accept(packet(4, 7000, 55), 16000).is_some());
        assert_eq!(r.acc, 5);
    }
    #[test]
    fn slow_board_clock_does_not_accumulate_delay() {
        let mut r = Receiver::default();
        // Board crystal 80 ppm slow against the host clock, 8 ms transit plus
        // up to 4 ms of arrival jitter, 30 minutes at 500 Hz.
        let mut rejected = 0;
        let mut worst = 0u32;
        for i in 0..900_000u64 {
            let board = i * 2000;
            let host = 10_000 + (board as f64 * 1.00008) as u64 + 8000 + (i % 3) * 2000;
            match r.accept(packet(i as u32 + 1, board as u32, (i % 1000) as i32), host) {
                Some(_) => worst = worst.max(r.delay_us),
                None => rejected += 1,
            }
        }
        assert_eq!(rejected, 0);
        assert!(worst < 6000, "apparent delay reached {worst} us");
    }
    #[test]
    fn socket_timeout_reconnect_stays_free_and_preserves_angle() {
        let board = UdpSocket::bind("127.0.0.1:0").unwrap();
        board
            .set_read_timeout(Some(Duration::from_millis(500)))
            .unwrap();
        let state = Arc::new(PlatterState::new());
        let cmds = Arc::new(CommandSlot::new());
        let link = WifiLink::open(
            &board.local_addr().unwrap().to_string(),
            state.clone(),
            cmds.clone(),
            None,
        )
        .unwrap();
        let mut b = [0; 2048];
        let (n, peer) = board.recv_from(&mut b).unwrap();
        let first = Packet::decode(&b[..n]).unwrap();
        assert_eq!(first.payload[2], Mode::Free as u8);
        let mut p = packet(1, 1000, 900);
        p.session = first.session;
        let mut batch = Vec::new();
        for i in 0..4 {
            let mut sample = packet(1 + i, 1000 + i * 2000, 900 + i as i32 * 4);
            sample.session = first.session;
            batch.extend_from_slice(&sample.encode());
        }
        board.send_to(&batch, peer).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        while state.snapshot().frames < 4 && std::time::Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(state.snapshot().connected);
        assert_eq!(state.snapshot().frames, 4);
        cmds.set(HostCommand {
            mode: Mode::Spin,
            ..Default::default()
        });
        let next = loop {
            let (n, _) = board.recv_from(&mut b).unwrap();
            let cmd = Packet::decode(&b[..n]).unwrap();
            if cmd.session != first.session {
                break cmd;
            }
            assert!(std::time::Instant::now() < deadline);
        };
        assert_eq!(next.payload[2], Mode::Free as u8);
        assert!(!state.snapshot().connected);
        assert_eq!(cmds.get().mode, Mode::Free);
        // Old-session packets cannot bring the link back up.
        board.send_to(&p.encode(), peer).unwrap();
        std::thread::sleep(Duration::from_millis(10));
        assert!(!state.snapshot().connected);
        p.session = next.session;
        p.payload = PlatterFrame {
            seq: 1,
            angle: -99999,
            velocity_mrad: 0,
            status: 2,
        }
        .encode();
        board.send_to(&p.encode(), peer).unwrap();
        while !state.snapshot().connected && std::time::Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(state.snapshot().connected);
        assert_eq!(state.snapshot().angle_units, 12);
        assert_eq!(cmds.get().mode, Mode::Free);
        drop(link);
    }
}
