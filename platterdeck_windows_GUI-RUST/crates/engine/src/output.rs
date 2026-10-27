//! cpal output sink. On Windows this is WASAPI shared mode: the callback
//! period is the device period (10 ms by default) whatever ring buffer is
//! requested; the requested size only sets the ring buffer, i.e. how much
//! scheduling slack the callback has before the DAC runs dry. The callback
//! thread joins the MMCSS "Audio" task class through cpal's `realtime`
//! feature. Exclusive mode / IAudioClient3 is not something cpal 0.18
//! offers.
use crate::deck::Deck;
use anyhow::{anyhow, Context, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{BufferSize, StreamConfig};
use std::sync::{Arc, Mutex};

/// Default ring buffer, seconds: two 10 ms WASAPI periods, so a whole period
/// of slack for the callback, and the first 20 ms of the Wi-Fi playback delay
/// budget ([`crate::deck::WIFI_BUFFER_US`] covers it plus sample transit).
pub const DEFAULT_BUFFER_S: f64 = 0.020;

pub struct Output {
    _stream: cpal::Stream,
    pub device_name: String,
    pub sample_rate: u32,
    pub channels: u16,
    pub host: String,
    /// What the stream reports as its buffer, if the backend says.
    pub buffer_frames: Option<u32>,
}

fn dev_name(d: &cpal::Device) -> Option<String> {
    d.description().ok().map(|desc| desc.name().to_string())
}

pub fn list_devices() -> Vec<String> {
    let host = cpal::default_host();
    host.output_devices()
        .map(|it| it.filter_map(|d| dev_name(&d)).collect())
        .unwrap_or_default()
}

pub fn open(deck: Deck, device_name: Option<&str>, buffer_frames: Option<u32>) -> Result<Output> {
    let host = cpal::default_host();
    let device = match device_name {
        Some(want) => {
            let want_lc = want.to_lowercase();
            host.output_devices()?
                .find(|d| dev_name(d).map(|n| n.to_lowercase().contains(&want_lc)).unwrap_or(false))
                .ok_or_else(|| anyhow!("no output device matching {want:?}"))?
        }
        None => host.default_output_device().ok_or_else(|| anyhow!("no default output device"))?,
    };
    let sup = device.default_output_config().context("default output config")?;
    let channels = sup.channels();
    let sample_rate = sup.sample_rate();
    let name = dev_name(&device).unwrap_or_else(|| "?".into());
    log::info!(
        "audio: {name} {sample_rate} Hz {channels} ch, supported buffer {:?}",
        sup.buffer_size()
    );

    // The callback owns the deck. The mutex is never contended; try_lock
    // keeps the callback from ever blocking.
    let deck = Arc::new(Mutex::new(deck));
    let build = |buffer_size: BufferSize| {
        let deck = deck.clone();
        let cfg = StreamConfig { channels, sample_rate, buffer_size };
        let ch = channels as usize;
        let rate = sample_rate as f64;
        device.build_output_stream(
            cfg,
            move |data: &mut [f32], info: &cpal::OutputCallbackInfo| {
                // When the first frame of this block reaches the DAC, relative
                // to now: ring-buffer fill plus the device's own latency. The
                // deck aims the platter timeline at that instant.
                let ts = info.timestamp();
                let latency_us = ts.playback.duration_since(ts.callback).as_micros() as u64;
                if let Ok(mut d) = deck.try_lock() {
                    d.render(data, ch, rate, latency_us);
                } else {
                    data.fill(0.0);
                }
            },
            |e| log::error!("audio stream: {e}"),
            None,
        )
    };

    let want = buffer_frames.unwrap_or((sample_rate as f64 * DEFAULT_BUFFER_S).round() as u32);
    let stream = match build(BufferSize::Fixed(want)) {
        Ok(s) => s,
        Err(e) => {
            log::warn!("audio: ring buffer of {want} frames refused ({e}); using device default");
            build(BufferSize::Default)?
        }
    };
    stream.play().context("start stream")?;
    let actual = stream.buffer_size().ok();
    log::info!("audio: stream buffer {:?} frames", actual);

    Ok(Output {
        _stream: stream,
        device_name: name,
        sample_rate,
        channels,
        host: format!("{:?}", host.id()),
        buffer_frames: actual,
    })
}
