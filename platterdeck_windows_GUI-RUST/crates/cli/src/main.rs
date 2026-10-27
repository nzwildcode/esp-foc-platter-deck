//! Wi-Fi platter diagnostics: frame rate, loss, jitter and motor status.
use anyhow::{anyhow, Result};
use clap::Parser;
use platter_engine::proto::{self, status, Mode};
use platter_engine::{tap, wifi, CommandSlot, HostCommand, PlatterState};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(name = "platter-cli", about = "PlatterDeck Wi-Fi frame check")]
struct Args {
    /// Board UDP endpoint (join its hotspot first).
    #[arg(long, default_value = "192.168.4.1:19704")]
    wifi: String,
    /// Command to hold: free | spin | brake | estop.
    #[arg(long, default_value = "free")]
    mode: String,
    /// SPIN target, rpm. Negative = reverse.
    #[arg(long, default_value_t = 33.333)]
    rpm: f32,
    /// Torque cap, volts (SimpleFOC voltage_limit).
    #[arg(long, default_value_t = 0.5)]
    torque: f32,
    /// Stop after this many seconds (0 = run until Ctrl-C).
    #[arg(long, default_value_t = 0)]
    secs: u64,
    /// Print interval, seconds.
    #[arg(long, default_value_t = 1.0)]
    interval: f32,
    /// Write every accepted platter sample to this CSV file (see engine tap).
    #[arg(long)]
    dump: Option<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mode = match args.mode.to_lowercase().as_str() {
        "free" => Mode::Free,
        "spin" => Mode::Spin,
        "brake" => Mode::Brake,
        "estop" => Mode::Estop,
        m => return Err(anyhow!("unknown mode {m:?}")),
    };
    let state = Arc::new(PlatterState::new());
    let cmds = Arc::new(CommandSlot::new());
    let (tap, _tap_writer) = match &args.dump {
        Some(path) => {
            let (t, w) = tap::start(path)?;
            println!("dumping samples to {}", path.display());
            (Some(t), Some(w))
        }
        None => (None, None),
    };
    let _wifi = wifi::WifiLink::open(&args.wifi, state.clone(), cmds.clone(), tap)?;
    let port = &args.wifi;
    // An explicit CLI --mode is applied once after the initial FREE handshake.
    // It is never automatically re-applied after a connection loss.
    let deadline = Instant::now() + Duration::from_secs(5);
    let ready = || {
        let s = state.snapshot();
        s.connected && s.status & status::LINK_TIMEOUT == 0 && status::mode(s.status) == Mode::Free
    };
    while !ready() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    if !ready() { return Err(anyhow!("no FREE handshake from {port}; join PlatterDeck Wi-Fi and check the address")); }
    cmds.set(HostCommand { mode, torque_v: args.torque, target_vel_mrad: proto::rpm_to_mrad(args.rpm), drag: 0 });
    println!(
        "{port}: holding {} target {:.2} rpm torque cap {:.2} V. Ctrl-C to stop.",
        mode.name(),
        args.rpm,
        args.torque
    );
    println!("{:>7} {:>6} {:>12} {:>8} {:>6} {:>7} {:>5}  status", "t(s)", "fps", "angle(rev)", "rpm", "crc", "dropped", "link");

    let t0 = Instant::now();
    let mut last = Instant::now();
    let mut last_frames = 0u64;
    let interval = Duration::from_secs_f32(args.interval.max(0.1));
    loop {
        std::thread::sleep(interval);
        let s = state.snapshot();
        let now = Instant::now();
        let dt = (now - last).as_secs_f64();
        let fps = (s.frames - last_frames) as f64 / dt;
        last = now;
        last_frames = s.frames;
        let st = s.status;
        let flags = format!(
            "{}{}{}{} {}",
            if st & status::ENABLED != 0 { "EN " } else { "-- " },
            if st & status::SENSOR_OK != 0 { "SENS " } else { "---- " },
            if st & status::FAULT != 0 { "FAULT " } else { "" },
            if st & status::LINK_TIMEOUT != 0 { "TIMEOUT " } else { "" },
            status::mode(st).name()
        );
        println!(
            "{:>7.1} {:>6.0} {:>12.4} {:>8.2} {:>6} {:>7} {:>5}  0x{:02X} {}",
            t0.elapsed().as_secs_f64(),
            fps,
            s.angle_rev(),
            s.rpm(),
            s.crc_errors,
            s.dropped,
            if s.connected { "up" } else { "DOWN" },
            st,
            flags
        );
        println!("        rejected={} jitter={:.2}ms max-gap={:.2}ms command={}",
            s.rejected, s.delay_us as f64 / 1000.0, s.max_gap_us as f64 / 1000.0, cmds.get().mode.name());
        if args.secs > 0 && t0.elapsed() >= Duration::from_secs(args.secs) {
            break;
        }
    }
    Ok(())
}
