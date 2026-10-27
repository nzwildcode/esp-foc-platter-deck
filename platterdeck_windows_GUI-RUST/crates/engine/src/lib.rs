//! PlatterDeck engine: platter source (Wi-Fi or simulator), whole-track
//! decoder, scratch deck, cpal output. UI-agnostic.
pub mod clock;
pub mod command;
pub mod deck;
pub mod decode;
pub mod output;
pub mod platter;
pub mod sim;
pub mod tap;
pub mod wifi;

pub use command::{CommandSlot, HostCommand};
pub use deck::{DeckCommand, DeckReadback};
pub use decode::Track;
pub use platter::{PlatterSnapshot, PlatterState};
pub use platter_proto as proto;

use anyhow::Result;
use crossbeam_channel::{Receiver, Sender};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicU64, Ordering};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Source {
    /// ESP32 platter via its Wi-Fi hotspot.
    Wifi(String),
    /// Mouse-driven software platter.
    Sim,
}

pub struct EngineConfig {
    pub source: Source,
    pub device: Option<String>,
    pub buffer_frames: Option<u32>,
    /// CSV debug tap of every platter sample and audio block (see [`tap`]).
    pub debug_log: Option<PathBuf>,
}

pub enum EngineEvent {
    TrackLoaded(Arc<Track>),
    LoadFailed(String),
}

/// Owns the platter source thread; dropping it stops the thread.
#[allow(dead_code)]
enum Link {
    Wifi(wifi::WifiLink),
    None,
    Sim(sim::SimPlatter),
}

pub struct EngineInfo {
    pub source: String,
    pub port: Option<String>,
    pub is_sim: bool,
    pub host: String,
    pub device: String,
    pub sample_rate: u32,
    pub channels: u16,
}

pub struct Engine {
    pub platter: Arc<PlatterState>,
    pub cmds: Arc<CommandSlot>,
    /// Mouse input for the simulator. Unused with a real link.
    pub hand: Arc<Mutex<sim::Hand>>,
    pub readback: Arc<DeckReadback>,
    pub track: Option<Arc<Track>>,
    pub info: EngineInfo,
    deck_tx: Sender<DeckCommand>,
    trash_rx: Receiver<Arc<Track>>,
    events_tx: Sender<(u64, EngineEvent)>,
    events_rx: Receiver<(u64, EngineEvent)>,
    load_generation: AtomicU64,
    _output: output::Output,
    link: Link,
    tap: Option<Arc<tap::Tap>>,
    _tap_writer: Option<tap::TapWriter>,
}

impl Engine {
    pub fn start(cfg: EngineConfig) -> Result<Engine> {
        let platter = Arc::new(PlatterState::new());
        let cmds = Arc::new(CommandSlot::new());
        let hand = Arc::new(Mutex::new(sim::Hand::default()));
        let readback = Arc::new(DeckReadback::default());
        let (deck_tx, deck_rx) = crossbeam_channel::bounded(64);
        let (trash_tx, trash_rx) = crossbeam_channel::bounded(8);
        let (events_tx, events_rx) = crossbeam_channel::unbounded();

        let (tap, tap_writer) = match &cfg.debug_log {
            Some(path) => {
                let (t, w) = tap::start(path)?;
                log::info!("debug tap: {}", path.display());
                (Some(t), Some(w))
            }
            None => (None, None),
        };
        let deck = deck::Deck::new(deck_rx, trash_tx, platter.clone(), readback.clone(), tap.clone());
        let out = output::open(deck, cfg.device.as_deref(), cfg.buffer_frames)?;

        let (link, port, source, is_sim) = Self::make_link(&cfg.source, &platter, &cmds, &hand, tap.clone())?;

        let info = EngineInfo {
            source,
            port,
            is_sim,
            host: out.host.clone(),
            device: out.device_name.clone(),
            sample_rate: out.sample_rate,
            channels: out.channels,
        };
        Ok(Engine {
            platter,
            cmds,
            hand,
            readback,
            track: None,
            info,
            deck_tx,
            trash_rx,
            events_tx,
            events_rx,
            load_generation: AtomicU64::new(0),
            _output: out,
            link,
            tap,
            _tap_writer: tap_writer,
        })
    }

    fn make_link(
        source: &Source,
        platter: &Arc<PlatterState>,
        cmds: &Arc<CommandSlot>,
        hand: &Arc<Mutex<sim::Hand>>,
        tap: Option<Arc<tap::Tap>>,
    ) -> Result<(Link, Option<String>, String, bool)> {
        platter.set_wifi(matches!(source, Source::Wifi(_)));
        match source {
            Source::Wifi(address) => {
                let l = wifi::WifiLink::open(address, platter.clone(), cmds.clone(), tap)?;
                Ok((Link::Wifi(l), Some(address.clone()), format!("Wi-Fi {address}"), false))
            }
            Source::Sim => Ok((
                Link::Sim(sim::SimPlatter::spawn(platter.clone(), cmds.clone(), hand.clone())),
                None,
                "simulator".to_string(),
                true,
            )),
        }
    }

    /// Switch platter source live. The platter angle carries over, so the
    /// playhead stays put. On failure the simulator takes over and the
    /// error comes back for the UI to show.
    pub fn set_source(&mut self, source: Source) -> Result<()> {
        // Stop the old source first: two writers on one platter state is nonsense.
        self.link = Link::None;
        self.platter.set_connected(false);
        self.cmds.set(HostCommand { mode: proto::Mode::Free, ..self.cmds.get() });
        self.platter.set_stats(0, 0);
        self.platter.set_wifi_stats(0, 0, 0);
        match Self::make_link(&source, &self.platter, &self.cmds, &self.hand, self.tap.clone()) {
            Ok((link, port, name, is_sim)) => {
                self.link = link;
                self.info.source = name;
                self.info.port = port;
                self.info.is_sim = is_sim;
                Ok(())
            }
            Err(e) => {
                let (link, port, name, is_sim) = Self::make_link(&Source::Sim, &self.platter, &self.cmds, &self.hand, None)?;
                self.link = link;
                self.info.source = name;
                self.info.port = port;
                self.info.is_sim = is_sim;
                Err(e)
            }
        }
    }

    /// Decode on a worker thread; the result arrives via [`Engine::poll`].
    pub fn load_track(&self, path: PathBuf) {
        let generation = self.load_generation.fetch_add(1, Ordering::Relaxed) + 1;
        let tx = self.events_tx.clone();
        std::thread::Builder::new()
            .name("platter-decode".into())
            .spawn(move || {
                // A decoder panic must surface as a failed load, not a hang.
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| decode::load(&path)));
                let ev = match result {
                    Ok(Ok(t)) => EngineEvent::TrackLoaded(Arc::new(t)),
                    Ok(Err(e)) => EngineEvent::LoadFailed(format!("{e:#}")),
                    Err(p) => {
                        let msg = p
                            .downcast_ref::<String>()
                            .cloned()
                            .or_else(|| p.downcast_ref::<&str>().map(|s| s.to_string()))
                            .unwrap_or_else(|| "decoder panicked".into());
                        EngineEvent::LoadFailed(format!("decoder panicked: {msg}"))
                    }
                };
                let _ = tx.send((generation, ev));
            })
            .expect("spawn decode thread");
    }

    /// Call from the UI every frame. Applies finished loads, frees
    /// buffers the audio thread handed back, returns what happened.
    pub fn poll(&mut self) -> Vec<EngineEvent> {
        while self.trash_rx.try_recv().is_ok() {}
        let mut evs = Vec::new();
        while let Ok((generation, ev)) = self.events_rx.try_recv() {
            if generation != self.load_generation.load(Ordering::Relaxed) { continue; }
            if let EngineEvent::TrackLoaded(t) = &ev {
                self.track = Some(t.clone());
                let _ = self.deck_tx.send(DeckCommand::Load(Some(t.clone())));
            }
            evs.push(ev);
        }
        evs
    }

    pub fn eject(&mut self) {
        self.load_generation.fetch_add(1, Ordering::Relaxed);
        self.track = None;
        let _ = self.deck_tx.send(DeckCommand::Load(None));
    }

    /// Needle drop at a track sample.
    pub fn jump(&self, sample: f64) {
        let _ = self.deck_tx.send(DeckCommand::Jump(sample));
    }

    pub fn set_gain(&self, gain: f32) {
        let _ = self.deck_tx.send(DeckCommand::Gain(gain));
    }

    pub fn set_extra_lead_us(&self, us: f64) {
        let _ = self.deck_tx.send(DeckCommand::ExtraLeadUs(us));
    }

    pub fn set_command(&self, cmd: HostCommand) {
        self.cmds.set(cmd);
        let _ = self.deck_tx.send(DeckCommand::Transport(cmd.mode));
    }

    /// Platter speed (33 1/3 or 45) and pitch. The platter speed only sets
    /// how much audio one revolution is; the tempo comes from the pitch.
    pub fn set_playback_speed(&self, platter_rpm: f64, pitch_pct: f64) {
        let _ = self.deck_tx.send(DeckCommand::PlatterRpm(platter_rpm));
        let _ = self.deck_tx.send(DeckCommand::Tempo(1.0 + pitch_pct / 100.0));
    }

    /// Steady playback uses the audio clock; vinyl follows encoder motion.
    pub fn set_steady_playback(&self, steady: bool) {
        let _ = self.deck_tx.send(DeckCommand::Steady(steady));
    }

    pub fn command(&self) -> HostCommand {
        self.cmds.get()
    }
}
