//! Debug tap: every accepted platter sample and every rendered audio block
//! as CSV lines, written by a background thread. Bench instrumentation for
//! separating link, encoder and motor problems; costs nothing when off.
//!
//! One record per line:
//! `S,t_us,angle_units,vel_mrad,status,delay_us,abs_angle` for a platter sample
//! (host-clock capture estimate, 1/65536 rev, board velocity, status byte,
//! excess link delay) and
//! `B,t_us,pos,rate,vel_rev_s,pos_err,lead_us,flags` for an audio block
//! (playhead in track samples, rate relative to nominal at block end,
//! least-squares platter velocity, position error the tracking loop saw,
//! platter-to-DAC delay, flags: r ready, s stale, t stationary, y steady,
//! h hands-on fast tracking rather than the flywheel estimate).
use anyhow::{Context, Result};
use crossbeam_channel::{bounded, Receiver, RecvTimeoutError, Sender};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug)]
pub enum Event {
    /// `abs_angle`: the board's own multi-turn angle, absolute since boot.
    Sample { t_us: u64, angle: i64, vel_mrad: i32, status: u8, delay_us: u32, abs_angle: i32 },
    Block {
        t_us: u64,
        pos: f64,
        rate: f64,
        vel_rev_s: f64,
        pos_err: f64,
        lead_us: u32,
        ready: bool,
        stale: bool,
        stationary: bool,
        steady: bool,
        hands: bool,
    },
}

pub struct Tap {
    tx: Sender<Event>,
    pub dropped: AtomicU32,
}

impl Tap {
    /// Never blocks and never allocates: safe from the audio callback. A
    /// full queue drops the event and counts it.
    pub fn send(&self, e: Event) {
        if self.tx.try_send(e).is_err() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Owns the writer thread; dropping it flushes and closes the file.
pub struct TapWriter {
    thread: Option<JoinHandle<()>>,
    stop: Arc<AtomicBool>,
}

impl Drop for TapWriter {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

pub fn start(path: &Path) -> Result<(Arc<Tap>, TapWriter)> {
    let file = File::create(path).with_context(|| format!("create {}", path.display()))?;
    let (tx, rx) = bounded(8192);
    let tap = Arc::new(Tap { tx, dropped: AtomicU32::new(0) });
    let stop = Arc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let thread = std::thread::Builder::new()
        .name("platter-tap".into())
        .spawn(move || run(file, rx, flag))
        .context("spawn tap writer")?;
    Ok((tap, TapWriter { thread: Some(thread), stop }))
}

fn run(file: File, rx: Receiver<Event>, stop: Arc<AtomicBool>) {
    let mut w = BufWriter::new(file);
    let _ = writeln!(
        w,
        "# PlatterDeck debug tap: S,t_us,angle_units,vel_mrad,status,delay_us,abs_angle | B,t_us,pos,rate,vel_rev_s,pos_err,lead_us,flags"
    );
    let mut flushed = Instant::now();
    while !stop.load(Ordering::Relaxed) {
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(e) => write(&mut w, e),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }
        // Flush often enough that a live reader sees fresh lines.
        if flushed.elapsed() >= Duration::from_millis(100) {
            let _ = w.flush();
            flushed = Instant::now();
        }
    }
    while let Ok(e) = rx.try_recv() {
        write(&mut w, e);
    }
    let _ = w.flush();
}

fn write(w: &mut BufWriter<File>, e: Event) {
    let _ = match e {
        Event::Sample { t_us, angle, vel_mrad, status, delay_us, abs_angle } => {
            writeln!(w, "S,{t_us},{angle},{vel_mrad},{status},{delay_us},{abs_angle}")
        }
        Event::Block { t_us, pos, rate, vel_rev_s, pos_err, lead_us, ready, stale, stationary, steady, hands } => {
            writeln!(
                w,
                "B,{t_us},{pos:.1},{rate:.5},{vel_rev_s:.5},{pos_err:.1},{lead_us},{}{}{}{}{}",
                if ready { "r" } else { "" },
                if stale { "s" } else { "" },
                if stationary { "t" } else { "" },
                if steady { "y" } else { "" },
                if hands { "h" } else { "" },
            )
        }
    };
}
