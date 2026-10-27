//! Whole-track decode to interleaved stereo f32 in RAM, plus a peak
//! overview for the waveform strip. Random access is then free, which is
//! what reverse scratch and backspin need.
use anyhow::{anyhow, Context, Result};
use std::path::Path;
use symphonia::core::codecs::audio::AudioDecoderOptions;
use symphonia::core::errors::Error;
use symphonia::core::formats::probe::Hint;
use symphonia::core::formats::{FormatOptions, TrackType};
use symphonia::core::io::MediaSourceStream;
use symphonia::core::meta::MetadataOptions;

pub const OVERVIEW_BINS: usize = 2048;

pub struct Track {
    pub name: String,
    pub sample_rate: u32,
    /// Stereo frames.
    pub frames: usize,
    /// Interleaved L R L R ..., `frames * 2` long.
    pub samples: Vec<f32>,
    /// Peak |sample| per bin, [`OVERVIEW_BINS`] long.
    pub overview: Vec<f32>,
}

impl Track {
    pub fn duration_s(&self) -> f64 {
        self.frames as f64 / self.sample_rate as f64
    }

    pub fn build_overview(&mut self) {
        let bins = OVERVIEW_BINS;
        let mut ov = vec![0f32; bins];
        if self.frames > 0 {
            let per_bin = (self.frames + bins - 1) / bins;
            for (b, slot) in ov.iter_mut().enumerate() {
                // Ceil-division means the last bins can start past the end.
                let start = (b * per_bin).min(self.frames);
                let end = ((b + 1) * per_bin).min(self.frames);
                let mut pk = 0f32;
                for s in &self.samples[start * 2..end * 2] {
                    pk = pk.max(s.abs());
                }
                *slot = pk;
            }
        }
        self.overview = ov;
    }
}

pub fn load(path: &Path) -> Result<Track> {
    let file = std::fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mss = MediaSourceStream::new(Box::new(file), Default::default());
    let mut hint = Hint::new();
    if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
        hint.with_extension(ext);
    }
    let mut format = symphonia::default::get_probe()
        .probe(&hint, mss, FormatOptions::default(), MetadataOptions::default())
        .context("probe container")?;

    let (track_id, params) = {
        let track = format.default_track(TrackType::Audio).ok_or_else(|| anyhow!("no audio track"))?;
        let params = track
            .codec_params
            .as_ref()
            .and_then(|p| p.audio())
            .ok_or_else(|| anyhow!("no audio codec parameters"))?
            .clone();
        (track.id, params)
    };
    let mut decoder = symphonia::default::get_codecs()
        .make_audio_decoder(&params, &AudioDecoderOptions::default())
        .context("make decoder")?;

    let mut sample_rate = params.sample_rate.unwrap_or(0);
    let mut channels = params.channels.as_ref().map(|c| c.count()).unwrap_or(0);
    let mut samples: Vec<f32> = Vec::new();
    let mut tmp: Vec<f32> = Vec::new();

    loop {
        let packet = match format.next_packet() {
            Ok(Some(p)) => p,
            Ok(None) | Err(Error::ResetRequired) => break,
            Err(Error::IoError(e)) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(e).context("read packet"),
        };
        if packet.track_id != track_id {
            continue;
        }
        let decoded = match decoder.decode(&packet) {
            Ok(d) => d,
            Err(Error::IoError(_)) | Err(Error::DecodeError(_)) => continue,
            Err(e) => return Err(e).context("decode"),
        };
        if decoded.is_empty() {
            continue;
        }
        let spec = decoded.spec();
        sample_rate = spec.rate();
        channels = spec.channels().count();
        tmp.clear();
        decoded.copy_to_vec_interleaved::<f32>(&mut tmp);
        match channels {
            0 => {}
            1 => {
                samples.reserve(tmp.len() * 2);
                for &s in &tmp {
                    samples.push(s);
                    samples.push(s);
                }
            }
            2 => samples.extend_from_slice(&tmp),
            n => {
                samples.reserve(tmp.len() / n * 2);
                for fr in tmp.chunks_exact(n) {
                    samples.push(fr[0]);
                    samples.push(fr[1]);
                }
            }
        }
    }

    if sample_rate == 0 || samples.is_empty() {
        return Err(anyhow!("decoded nothing from {}", path.display()));
    }
    let frames = samples.len() / 2;
    let mut track = Track {
        name: path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default(),
        sample_rate,
        frames,
        samples,
        overview: Vec::new(),
    };
    track.build_overview();
    log::info!(
        "decoded {}: {} Hz, {} ch in, {:.1} s",
        track.name,
        sample_rate,
        channels,
        track.duration_s()
    );
    Ok(track)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn track_of(frames: usize) -> Track {
        Track {
            name: "t".into(),
            sample_rate: 48_000,
            frames,
            samples: (0..frames * 2).map(|i| if i % 2 == 0 { 0.5 } else { -0.25 }).collect(),
            overview: vec![],
        }
    }

    #[test]
    fn overview_handles_any_length() {
        // 288000 = 6 s at 48 kHz: not a multiple of the bin count, the case
        // that indexed past the end on the first self-test.
        for frames in [0, 1, 7, 2047, 2048, 2049, 288_000, 288_001] {
            let mut t = track_of(frames);
            t.build_overview();
            assert_eq!(t.overview.len(), OVERVIEW_BINS);
            if frames > 0 {
                assert_eq!(t.overview[0], 0.5);
                assert!(t.overview.iter().all(|&p| p == 0.5 || p == 0.0));
            }
        }
    }
}
