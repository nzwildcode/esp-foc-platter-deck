#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
//! platterdeck: Wi-Fi FOC platter (or the mouse
//! simulator), audio out via cpal, egui front end.
//!
//! Release builds are a GUI-subsystem exe with no console window;
//! `sys::attach_parent_console` keeps `--list` / `--selftest` / logs visible
//! when launched from a shell.
mod sys;
mod ui;

use anyhow::{anyhow, bail, Result};
use clap::Parser;
use platter_engine::proto::{Mode, RPM33_MRAD};
use platter_engine::{output, Engine, EngineConfig, EngineEvent, HostCommand, Source};
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(name = "platterdeck", about = "Motorised platter DJ deck, PC side")]
struct Args {
    /// Use the mouse-driven simulated platter.
    #[arg(long)]
    sim: bool,
    /// Wi-Fi board endpoint; the PC must first join the PlatterDeck hotspot.
    #[arg(long, default_value = "192.168.4.1:19704")]
    wifi: String,
    /// Output device name substring (default: system default).
    #[arg(long)]
    device: Option<String>,
    /// Output ring buffer in frames (default 20 ms). WASAPI shared mode
    /// keeps its 10 ms callback period; this only sets the slack.
    #[arg(long)]
    buffer: Option<u32>,
    /// List audio output devices, then exit.
    #[arg(long)]
    list: bool,
    /// Headless check: start the engine, SPIN at 33 1/3, load the file if
    /// given, run for --secs, print what the audio thread saw, exit.
    /// Output gain is 0 so it stays silent.
    #[arg(long)]
    selftest: bool,
    /// Follow physical platter motion for scratching (default: steady tempo).
    #[arg(long)]
    vinyl: bool,
    /// Self-test duration, seconds.
    #[arg(long, default_value_t = 3.0)]
    secs: f32,
    /// Write every platter sample and audio block to this CSV for bench
    /// debugging (tools/platter_log.py reads it live).
    #[arg(long)]
    debug_log: Option<PathBuf>,
    /// Track to load at start.
    file: Option<PathBuf>,
}

fn main() -> Result<()> {
    sys::attach_parent_console();
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    if args.list {
        println!("Audio outputs:");
        for d in output::list_devices() {
            println!("  {d}");
        }
        return Ok(());
    }

    let source = if args.sim { Source::Sim } else { Source::Wifi(args.wifi.clone()) };

    let engine = Engine::start(EngineConfig {
        source,
        device: args.device.clone(),
        buffer_frames: args.buffer,
        debug_log: args.debug_log.clone(),
    })?;
    engine.set_steady_playback(!args.vinyl);
    log::info!(
        "platter: {}  audio: {} via {} @ {} Hz x{}",
        engine.info.source,
        engine.info.device,
        engine.info.host,
        engine.info.sample_rate,
        engine.info.channels
    );

    if args.selftest {
        return selftest(engine, args.file.clone(), args.secs, !args.vinyl);
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(format!("PlatterDeck {}", env!("CARGO_PKG_VERSION")))
            .with_app_id("PlatterDeck")
            .with_inner_size([1100.0, 720.0])
            .with_min_inner_size([800.0, 560.0])
            .with_icon(window_icon()),
        renderer: eframe::Renderer::Glow,
        ..Default::default()
    };
    let file = args.file.clone();
    eframe::run_native(
        "PlatterDeck",
        options,
        Box::new(move |cc| {
            ui::apply_skin(&cc.egui_ctx);
            Ok(Box::new(ui::App::new(engine, file, !args.vinyl)))
        }),
    )
    .map_err(|e| anyhow!("eframe: {e}"))
}

/// 64x64 vinyl disc drawn in code, so there is no icon asset to ship.
fn window_icon() -> egui::IconData {
    const N: usize = 64;
    let mut rgba = vec![0u8; N * N * 4];
    let c = (N as f32 - 1.0) / 2.0;
    for y in 0..N {
        for x in 0..N {
            let dx = x as f32 - c;
            let dy = y as f32 - c;
            let r = (dx * dx + dy * dy).sqrt();
            let px = if r > 31.0 {
                [0, 0, 0, 0]
            } else if r > 30.0 {
                [90, 90, 90, 255]
            } else if r < 3.0 {
                [20, 20, 20, 255]
            } else if r < 11.0 {
                [178, 44, 44, 255]
            } else if ((r as i32) % 4 == 0) || (dx.abs() < 1.2 && dy < 0.0) {
                [70, 70, 70, 255]
            } else {
                [28, 28, 28, 255]
            };
            rgba[(y * N + x) * 4..(y * N + x) * 4 + 4].copy_from_slice(&px);
        }
    }
    egui::IconData { rgba, width: N as u32, height: N as u32 }
}

fn selftest(mut engine: Engine, file: Option<PathBuf>, secs: f32, steady: bool) -> Result<()> {
    engine.set_gain(0.0);
    if !engine.info.is_sim {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !engine.platter.snapshot().connected && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        if !engine.platter.snapshot().connected { bail!("platter connection not established"); }
    }
    // Same torque cap as the deck's default: the board's cogging compensation
    // needs the headroom, and a lower cap would clip it.
    engine.set_command(HostCommand { mode: Mode::Spin, torque_v: 1.0, target_vel_mrad: RPM33_MRAD, drag: 0 });
    let want_track = file.is_some();
    if let Some(f) = file {
        engine.load_track(f);
    }
    let t0 = Instant::now();
    let mut next_print = Duration::ZERO;
    let mut loaded = false;
    let mut track_rate = 0u32;
    let mut last_rate = 0.0;
    // (elapsed, platter angle, playhead) at mid-run: the second half gives
    // mean rates that the encoder noise and residual ripple cannot fool.
    let mut mid: Option<(f32, f64, f64)> = None;
    while t0.elapsed().as_secs_f32() < secs {
        for ev in engine.poll() {
            match ev {
                EngineEvent::TrackLoaded(t) => {
                    println!(
                        "loaded {} : {} Hz, {} frames, {:.2} s, overview {} bins",
                        t.name,
                        t.sample_rate,
                        t.frames,
                        t.duration_s(),
                        t.overview.len()
                    );
                    loaded = true;
                    track_rate = t.sample_rate;
                }
                EngineEvent::LoadFailed(e) => bail!("load failed: {e}"),
            }
        }
        if t0.elapsed() >= next_print {
            let p = engine.platter.snapshot();
            let rb = &engine.readback;
            last_rate = rb.rate();
            if mid.is_none() && loaded && t0.elapsed().as_secs_f32() >= secs * 0.5 {
                mid = Some((t0.elapsed().as_secs_f32(), p.angle_rev(), rb.pos()));
            }
            println!(
                "t={:.1}s platter {:>8.3} rev {:>6.2} rpm link={}  audio cb={} block={} fr lead={} us  pos={:.0} rate={:.3}x",
                t0.elapsed().as_secs_f32(),
                p.angle_rev(),
                p.rpm(),
                p.connected,
                rb.callbacks.load(Ordering::Relaxed),
                rb.block_frames.load(Ordering::Relaxed),
                rb.lead_us.load(Ordering::Relaxed),
                rb.pos(),
                last_rate
            );
            next_print += Duration::from_millis(500);
        }
        std::thread::sleep(Duration::from_millis(50));
    }

    let rb = &engine.readback;
    let cbs = rb.callbacks.load(Ordering::Relaxed);
    let block = rb.block_frames.load(Ordering::Relaxed);
    let cb_hz = cbs as f32 / secs;
    println!(
        "summary: {} callbacks in {:.1} s = {:.0}/s, block {} frames = {:.2} ms at {} Hz",
        cbs,
        secs,
        cb_hz,
        block,
        block as f32 * 1000.0 / engine.info.sample_rate as f32,
        engine.info.sample_rate
    );
    if cbs == 0 {
        bail!("audio callback never ran");
    }
    if want_track {
        if !loaded {
            bail!("track did not finish loading in {secs} s");
        }
        let p = engine.platter.snapshot();
        let (t_mid, a_mid, pos_mid) = mid.ok_or_else(|| anyhow!("run too short for a mean-rate check"))?;
        let dt = (t0.elapsed().as_secs_f32() - t_mid) as f64;
        let rpm_mean = (p.angle_rev() - a_mid) / dt * 60.0;
        let mean_rate = (rb.pos() - pos_mid) / (dt * track_rate.max(1) as f64);
        let expect_rate = if steady { 1.0 } else { rpm_mean / (100.0 / 3.0) };
        println!(
            "mode={} platter mean {:.2} rpm over the last {:.1} s -> expected rate {:.3}x, deck mean {:.3}x (last block {:.3}x)",
            if steady { "STEADY" } else { "VINYL" }, rpm_mean, dt, expect_rate, mean_rate, last_rate
        );
        if (mean_rate - expect_rate).abs() > 0.02 {
            bail!("deck mean rate {mean_rate:.3} does not track platter ({expect_rate:.3})");
        }
        if rb.pos() <= 0.0 {
            bail!("playhead did not advance");
        }
    }
    println!("SELFTEST OK");
    Ok(())
}
