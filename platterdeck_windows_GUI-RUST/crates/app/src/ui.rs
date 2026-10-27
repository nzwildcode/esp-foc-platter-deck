//! egui front end, skinned like a CDJ / SL-1200: dark deck, strobe-dot
//! platter, LCD-style display, vertical pitch fader (+ at the bottom, as on
//! the hardware), green PLAY, orange CUE, red STOP, four coloured hot-cue
//! pads, scrolling zoomed waveform plus an overview strip.
use egui::{
    Align2, Color32, CornerRadius, FontId, Key, Margin, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Vec2,
};
use platter_engine::proto::{self, status, Mode};
use platter_engine::{Engine, EngineEvent, HostCommand, PlatterSnapshot, Source};
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

const RPM33: f32 = 100.0 / 3.0;
const RPM45: f32 = 45.0;

// Palette
const BG: Color32 = Color32::from_rgb(14, 15, 18);
const PANEL: Color32 = Color32::from_rgb(24, 26, 31);
const PANEL_EDGE: Color32 = Color32::from_rgb(48, 52, 60);
const LCD_BG: Color32 = Color32::from_rgb(8, 22, 30);
const LCD_TEXT: Color32 = Color32::from_rgb(92, 214, 255);
const LCD_DIM: Color32 = Color32::from_rgb(40, 110, 140);
const TEXT: Color32 = Color32::from_rgb(210, 214, 220);
const TEXT_DIM: Color32 = Color32::from_rgb(120, 126, 136);
const GREEN: Color32 = Color32::from_rgb(46, 204, 113);
const ORANGE: Color32 = Color32::from_rgb(255, 140, 26);
const RED: Color32 = Color32::from_rgb(229, 57, 53);
const BLUE: Color32 = Color32::from_rgb(64, 156, 255);
const PINK: Color32 = Color32::from_rgb(233, 84, 200);
const PAD_COLORS: [Color32; 4] = [GREEN, ORANGE, BLUE, PINK];
const WAVE_LO: Color32 = Color32::from_rgb(30, 110, 190);
const WAVE_HI: Color32 = Color32::from_rgb(140, 225, 255);
const PLAYHEAD: Color32 = Color32::from_rgb(255, 59, 48);

/// Dark deck skin. Called once from the eframe creator.
pub fn apply_skin(ctx: &egui::Context) {
    let mut v = egui::Visuals::dark();
    v.panel_fill = BG;
    v.window_fill = PANEL;
    v.extreme_bg_color = LCD_BG;
    v.faint_bg_color = PANEL;
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = ORANGE.gamma_multiply(0.55);
    v.selection.stroke = Stroke::new(1.0, ORANGE);
    v.widgets.noninteractive.bg_fill = PANEL;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, PANEL_EDGE);
    v.widgets.inactive.bg_fill = Color32::from_rgb(40, 44, 52);
    v.widgets.inactive.weak_bg_fill = Color32::from_rgb(34, 37, 44);
    v.widgets.hovered.bg_fill = Color32::from_rgb(56, 61, 72);
    v.widgets.hovered.weak_bg_fill = Color32::from_rgb(48, 52, 62);
    v.widgets.active.bg_fill = Color32::from_rgb(70, 76, 90);
    v.widgets.active.weak_bg_fill = Color32::from_rgb(60, 66, 78);
    for w in [
        &mut v.widgets.noninteractive,
        &mut v.widgets.inactive,
        &mut v.widgets.hovered,
        &mut v.widgets.active,
        &mut v.widgets.open,
    ] {
        w.corner_radius = CornerRadius::same(4);
    }
    v.slider_trailing_fill = true;
    ctx.all_styles_mut(|s| {
        s.visuals = v.clone();
        s.text_styles.insert(egui::TextStyle::Monospace, FontId::monospace(14.0));
        s.text_styles.insert(egui::TextStyle::Body, FontId::proportional(14.0));
        s.text_styles.insert(egui::TextStyle::Button, FontId::proportional(14.0));
        s.spacing.item_spacing = Vec2::new(8.0, 6.0);
        s.spacing.slider_width = 150.0;
    });
}

/// What the PLATTER picker points at.
#[derive(Clone, PartialEq, Eq)]
enum SourceChoice {
    Wifi,
    Sim,
}

pub struct App {
    engine: Engine,
    src_choice: SourceChoice,
    wifi_address: String,
    path_text: String,
    recent_tracks: Vec<PathBuf>,
    status_line: String,
    mode: Mode,
    base_rpm: f32,
    pitch_pct: f32,
    torque_v: f32,
    gain: f32,
    steady_playback: bool,
    lead_ms: f32,
    /// Pitch fader range, percent: 8, 16 or 50.
    pitch_range: f32,
    /// Momentary pitch bend while a nudge button is held, percent.
    bend_pct: f32,
    /// Platter start and brake times, seconds (0 = as fast as the motor goes).
    start_s: f32,
    brake_s: f32,
    /// Start/brake ramp, 0..1, applied to the commanded speed and the tempo.
    ramp: f32,
    ramp_t: Instant,
    /// PAUSE pressed with a brake time set: winding down before BRAKE.
    braking: bool,
    cues: [Option<f64>; 4],
    // simulator hand
    finger_last: Option<f64>,
    finger_unwrapped: f64,
    // rate meters
    meter_t: Instant,
    meter_frames: u64,
    meter_callbacks: u64,
    platter_fps: f32,
    callback_hz: f32,
    // level meter ballistics
    meter_l: f32,
    meter_r: f32,
}

/// HELP drop-down: how to operate the deck, in the order things go wrong.
fn help_menu(ui: &mut egui::Ui) {
    ui.set_min_width(440.0);
    let head = |ui: &mut egui::Ui, t: &str| {
        ui.add_space(4.0);
        ui.label(RichText::new(t).strong().color(LCD_TEXT));
    };
    let line = |ui: &mut egui::Ui, t: &str| {
        ui.label(RichText::new(t).color(TEXT));
    };
    ui.label(RichText::new("HOW TO OPERATE PLATTERDECK").size(15.0).strong().color(ORANGE));
    head(ui, "1. Connect");
    line(ui, "Join the PlatterDeck Wi-Fi hotspot (password 12345678), then press Connect.");
    line(ui, "LINK UP and about 500 fps in the top bar mean the board is talking.");
    head(ui, "2. Load a track");
    line(ui, "Open track..., Choose previous track, or drop an audio file on the window.");
    head(ui, "3. Start the motor");
    line(ui, "Press FREE MOTOR (key F) first, then PLAY / SPIN. Space toggles PLAY / PAUSE.");
    line(ui, "After STOP / E-STOP the board latches: FOC: ESTOP in the top bar. FREE MOTOR releases it.");
    line(ui, "If it stays latched, press Disconnect, Connect, FREE MOTOR, PLAY.");
    head(ui, "4. STEADY or VINYL");
    line(ui, "STEADY: music runs from the audio clock at the pitch-fader tempo; the platter just spins.");
    line(ui, "VINYL: the platter drives the music. Touch, slow, stop or reverse it to scratch.");
    head(ui, "5. Speed and pitch");
    line(ui, "33 / 45 set the platter speed only; the track plays at 1.00x on either.");
    line(ui, "The pitch fader (+ at the bottom, as on a turntable) sets tempo and motor speed together.");
    line(ui, "Under it: 0 resets, the range button steps 8 / 16 / 50 %, and holding - or + bends the pitch 4 %");
    line(ui, "to nudge the beat into place.");
    head(ui, "DECK knobs");
    line(ui, "GAIN: output level. START / BRAKE: how long the platter takes to wind up after PLAY and down");
    line(ui, "after PAUSE (fast = motor limit). Drag a knob up or down; double-click resets it.");
    head(ui, "6. Cues");
    line(ui, "CUE is hot cue 1. Pads 1-4 (keys 1-4): click to jump, click an empty pad to set,");
    line(ui, "shift+click to move, right-click to clear.");
    head(ui, "7. Waveforms");
    line(ui, "Top: 8 s around the needle (red line), scrolls while playing. Bottom: whole track, click to seek.");
    line(ui, "Orange = played, blue = still to come.");
    head(ui, "8. Something wrong?");
    line(ui, "fps far below 500 or Wi-Fi jitter above 20 ms: adapter power saving; set it to Maximum Performance.");
    line(ui, "Platter delay 40 ms is the scratch latency; raise extra lead if scratches feel late.");
    line(ui, "The board keeps its tuning and cogging compensation in flash; nothing to set here.");
    ui.add_space(8.0);
    ui.separator();
    ui.label(RichText::new("Made by Dr. Kishor Kumar").strong().color(ORANGE));
    ui.label(RichText::new("ESP32 FOC platter + Windows deck, 2026").small().color(TEXT_DIM));
}

impl App {
    pub fn new(engine: Engine, file: Option<PathBuf>, steady_playback: bool) -> Self {
        let cmd = engine.command();
        let src_choice = if engine.info.is_sim { SourceChoice::Sim } else { SourceChoice::Wifi };
        let wifi_address = if src_choice == SourceChoice::Wifi {
            engine.info.port.clone().unwrap_or_else(|| proto::wifi::DEFAULT_ADDR.into())
        } else { proto::wifi::DEFAULT_ADDR.into() };
        let mut app = App {
            engine,
            src_choice,
            wifi_address,
            path_text: file.as_ref().map(|p| p.display().to_string()).unwrap_or_default(),
            recent_tracks: Vec::new(),
            status_line: "ready".into(),
            mode: cmd.mode,
            base_rpm: RPM33,
            pitch_pct: 0.0,
            torque_v: cmd.torque_v,
            gain: 1.0,
            steady_playback,
            lead_ms: 0.0,
            pitch_range: 8.0,
            bend_pct: 0.0,
            start_s: 0.0,
            brake_s: 0.0,
            ramp: if cmd.mode == Mode::Spin { 1.0 } else { 0.0 },
            ramp_t: Instant::now(),
            braking: false,
            cues: [None; 4],
            finger_last: None,
            finger_unwrapped: 0.0,
            meter_t: Instant::now(),
            meter_frames: 0,
            meter_callbacks: 0,
            platter_fps: 0.0,
            callback_hz: 0.0,
            meter_l: 0.0,
            meter_r: 0.0,
        };
        app.push_command();
        app.engine.set_steady_playback(steady_playback);
        if let Some(f) = file {
            app.load(f);
        }
        app
    }

    fn target_rpm(&self) -> f32 {
        self.base_rpm * (1.0 + self.pitch_pct / 100.0)
    }

    /// Platter speed for display: least squares over the last 250 ms of
    /// encoder angles. The board's per-sample velocity field carries about
    /// +-2 rpm of estimator noise and is not a measurement of the platter.
    fn display_rpm(&self, p: &PlatterSnapshot) -> f64 {
        self.engine
            .platter
            .velocity_at(platter_engine::clock::now_us(), 250_000)
            .map(|rev_s| rev_s * 60.0)
            .unwrap_or_else(|| p.rpm())
    }

    /// Speed actually commanded relative to the 33/45 nominal: pitch fader,
    /// pitch bend and the start/brake ramp.
    fn speed_factor(&self) -> f32 {
        (1.0 + self.pitch_pct / 100.0) * (1.0 + self.bend_pct / 100.0) * self.ramp
    }

    fn push_command(&self) {
        let factor = self.speed_factor();
        self.engine.set_command(HostCommand {
            mode: self.mode,
            torque_v: self.torque_v,
            target_vel_mrad: proto::rpm_to_mrad(self.base_rpm * factor),
            drag: 0,
        });
        // The same factor is the STEADY tempo and VINYL's commanded speed, so
        // a wind-up or wind-down is heard in both modes.
        self.engine.set_playback_speed(self.base_rpm as f64, (factor as f64 - 1.0) * 100.0);
    }

    fn set_mode(&mut self, m: Mode) {
        if self.mode != m {
            self.mode = m;
            self.push_command();
        }
    }

    /// PLAY / PAUSE with the START and BRAKE times applied.
    fn toggle_play(&mut self) {
        if self.mode == Mode::Spin && !self.braking {
            if self.brake_s > 0.01 {
                self.braking = true; // wind down, then BRAKE
            } else {
                self.set_mode(Mode::Brake);
            }
        } else if self.mode == Mode::Spin {
            self.braking = false; // pressed again while winding down: back up to speed
        } else {
            self.ramp = if self.start_s > 0.01 { 0.0 } else { 1.0 };
            self.ramp_t = Instant::now();
            self.set_mode(Mode::Spin);
        }
    }

    /// Advance the start/brake ramp once per frame while spinning.
    fn update_ramp(&mut self) {
        let now = Instant::now();
        let dt = now.duration_since(self.ramp_t).as_secs_f32().min(0.1);
        self.ramp_t = now;
        if self.mode != Mode::Spin {
            self.braking = false;
            return;
        }
        let before = self.ramp;
        let (goal, secs) = if self.braking { (0.0, self.brake_s) } else { (1.0, self.start_s) };
        self.ramp = if secs <= 0.01 {
            goal
        } else if goal > self.ramp {
            (self.ramp + dt / secs).min(1.0)
        } else {
            (self.ramp - dt / secs).max(0.0)
        };
        if self.braking && self.ramp <= 0.0 {
            self.braking = false;
            self.set_mode(Mode::Brake);
        } else if (self.ramp - before).abs() > 1e-4 {
            self.push_command();
        }
    }

    fn load(&mut self, path: PathBuf) {
        if !self.recent_tracks.contains(&path) { self.recent_tracks.push(path.clone()); }
        self.status_line = format!("loading {}", path.display());
        self.path_text = path.display().to_string();
        self.engine.load_track(path);
    }

    fn cue_press(&mut self, i: usize, shift: bool) {
        let pos = self.engine.readback.pos();
        match (self.cues[i], shift) {
            (Some(s), false) => self.engine.jump(s),
            _ => self.cues[i] = Some(pos),
        }
    }

    fn update_meters(&mut self, p: &PlatterSnapshot) {
        let dt = self.meter_t.elapsed().as_secs_f32();
        if dt >= 0.5 {
            let cbs = self.engine.readback.callbacks.load(Ordering::Relaxed);
            self.platter_fps = (p.frames - self.meter_frames) as f32 / dt;
            self.callback_hz = (cbs - self.meter_callbacks) as f32 / dt;
            self.meter_frames = p.frames;
            self.meter_callbacks = cbs;
            self.meter_t = Instant::now();
        }
        // Level meter: instant attack, ~0.3 s release at 60 fps.
        let (l, r) = self.engine.readback.peaks();
        self.meter_l = if l > self.meter_l { l } else { self.meter_l * 0.93 };
        self.meter_r = if r > self.meter_r { r } else { self.meter_r * 0.93 };
    }

    fn handle_keys(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() { return; }
        let (space, f_key, shift, digits) = ctx.input(|i| {
            (
                i.key_pressed(Key::Space),
                i.key_pressed(Key::F),
                i.modifiers.shift,
                [
                    i.key_pressed(Key::Num1),
                    i.key_pressed(Key::Num2),
                    i.key_pressed(Key::Num3),
                    i.key_pressed(Key::Num4),
                ],
            )
        });
        if space {
            self.toggle_play();
        }
        if f_key {
            self.set_mode(Mode::Free);
        }
        for (i, hit) in digits.iter().enumerate() {
            if *hit {
                self.cue_press(i, shift);
            }
        }
    }

    // ---------------------------------------------------------------- top bar

    fn top_bar(&mut self, ui: &mut egui::Ui, p: &PlatterSnapshot) {
        ui.horizontal_wrapped(|ui| {
            let link_col = if p.connected { GREEN } else { RED };
            ui.label(RichText::new(if p.connected { "LINK UP" } else { "LINK DOWN" }).color(link_col).strong());
            ui.label(RichText::new(&self.engine.info.source).color(TEXT_DIM));
            ui.separator();
            ui.label(
                RichText::new(format!(
                    "{:>5.0} fps  {:>9.3} rev  {:>7.2} rpm  crc {}  drop {}",
                    self.platter_fps,
                    p.angle_rev(),
                    self.display_rpm(p),
                    p.crc_errors,
                    p.dropped
                ))
                .monospace()
                .color(TEXT_DIM),
            );
            ui.separator();
            let st = p.status;
            ui.label(
                RichText::new(format!(
                    "FOC: {} {}{}{}{}",
                    status::mode(st).name(),
                    if st & status::ENABLED != 0 { "EN " } else { "" },
                    if st & status::SENSOR_OK != 0 { "SENS " } else { "" },
                    if st & status::FAULT != 0 { "FAULT " } else { "" },
                    if st & status::LINK_TIMEOUT != 0 { "TIMEOUT " } else { "" },
                ))
                .monospace()
                .color(if st & (status::FAULT | status::LINK_TIMEOUT) != 0 { RED } else { TEXT_DIM }),
            );
            ui.separator();
            let rb = &self.engine.readback;
            let block = rb.block_frames.load(Ordering::Relaxed);
            ui.label(
                RichText::new(format!(
                    "{} {} Hz  block {} fr ({:.1} ms)  cb {:.0}/s  {} {:.1} ms",
                    self.engine.info.device,
                    self.engine.info.sample_rate,
                    block,
                    block as f32 * 1000.0 / self.engine.info.sample_rate.max(1) as f32,
                    self.callback_hz,
                    if p.wifi { "platter delay" } else { "lead" },
                    rb.lead_us.load(Ordering::Relaxed) as f32 / 1000.0,
                ))
                .monospace()
                .color(TEXT_DIM),
            );
        });
    }

    // ---------------------------------------------------------------- platter

    fn platter(&mut self, ui: &mut egui::Ui, p: &PlatterSnapshot) {
        // Leave room below the disc for 33/45 and motor release controls.
        let size = ui.available_width().min((ui.available_height() - 48.0).max(100.0)).min(480.0);
        let (resp, painter) = ui.allocate_painter(Vec2::splat(size), Sense::click_and_drag());
        let c = resp.rect.center();
        let r = size * 0.47;
        let rev = if self.steady_playback {
            self.engine.track.as_ref().map(|t| self.engine.readback.pos() /
                (t.sample_rate as f64 * platter_engine::deck::SECONDS_PER_REV)).unwrap_or(0.0)
        } else {
            self.engine.platter.angle_at(platter_engine::clock::now_us().saturating_sub(
                if p.wifi { platter_engine::deck::WIFI_BUFFER_US } else { 0 })).unwrap_or(p.angle_rev())
        };
        let ang = rev * std::f64::consts::TAU;
        let dir = |a: f64| Vec2::new(a.sin() as f32, -(a.cos() as f32));

        // Deck plinth, platter rim, slipmat grooves.
        painter.circle_filled(c, r + 6.0, Color32::from_rgb(38, 40, 46));
        painter.circle_filled(c, r, Color32::from_rgb(22, 22, 25));
        painter.circle_stroke(c, r, Stroke::new(1.5, Color32::from_rgb(70, 72, 80)));
        for k in 0..14 {
            let rr = r * (0.40 + 0.04 * k as f32);
            painter.circle_stroke(c, rr, Stroke::new(0.6, Color32::from_rgb(34, 34, 38)));
        }
        // Strobe dots on the rim, rotating with the platter.
        let dots = 96;
        for k in 0..dots {
            let a = ang + k as f64 * std::f64::consts::TAU / dots as f64;
            let col = if k % 8 == 0 { Color32::from_rgb(200, 200, 200) } else { Color32::from_rgb(110, 110, 115) };
            painter.circle_filled(c + dir(a) * (r * 0.955), if k % 8 == 0 { 2.2 } else { 1.4 }, col);
        }
        // Position marker: one bright orange spoke, two dim ones.
        for k in 0..3 {
            let a = ang + k as f64 * std::f64::consts::TAU / 3.0;
            let (w, col) = if k == 0 { (4.0, ORANGE) } else { (1.5, Color32::from_rgb(80, 82, 90)) };
            painter.line_segment([c + dir(a) * (r * 0.36), c + dir(a) * (r * 0.92)], Stroke::new(w, col));
        }
        // Label with the track name, rotating text is overkill: keep it upright.
        painter.circle_filled(c, r * 0.33, Color32::from_rgb(150, 34, 34));
        painter.circle_stroke(c, r * 0.33, Stroke::new(1.0, Color32::from_rgb(90, 20, 20)));
        painter.circle_filled(c, r * 0.025, Color32::from_rgb(20, 20, 20));
        let label = self
            .engine
            .track
            .as_ref()
            .map(|t| t.name.chars().take(18).collect::<String>())
            .unwrap_or_else(|| "NO TRACK".into());
        painter.text(c + Vec2::new(0.0, -r * 0.12), Align2::CENTER_CENTER, label, FontId::proportional(11.0), Color32::from_rgb(240, 220, 200));
        painter.text(
            c + Vec2::new(0.0, r * 0.13),
            Align2::CENTER_CENTER,
            format!("{:.1} rpm", self.display_rpm(p)),
            FontId::monospace(15.0),
            Color32::WHITE,
        );
        if self.engine.info.is_sim {
            painter.text(
                Pos2::new(c.x, resp.rect.bottom() - 2.0),
                Align2::CENTER_BOTTOM,
                "simulated platter: drag the disc",
                FontId::proportional(11.0),
                TEXT_DIM,
            );
            // Finger angle in revs, same convention as the marker (0 = up, clockwise).
            let finger = |pos: Pos2| -> f64 {
                let d = pos - c;
                (d.x as f64).atan2(-(d.y as f64)) / std::f64::consts::TAU
            };
            if resp.drag_started() {
                if let Some(pos) = resp.interact_pointer_pos() {
                    let f = finger(pos);
                    self.finger_unwrapped = f;
                    self.finger_last = Some(f);
                    self.engine.hand.lock().unwrap().grab(f);
                }
            } else if resp.dragged() {
                if let Some(pos) = resp.interact_pointer_pos() {
                    let f = finger(pos);
                    if let Some(last) = self.finger_last {
                        let mut d = f - last;
                        if d > 0.5 {
                            d -= 1.0;
                        } else if d < -0.5 {
                            d += 1.0;
                        }
                        self.finger_unwrapped += d;
                    }
                    self.finger_last = Some(f);
                    self.engine.hand.lock().unwrap().drag(self.finger_unwrapped);
                }
            }
            if resp.drag_stopped() {
                self.engine.hand.lock().unwrap().release();
                self.finger_last = None;
            }
        }
    }

    fn speed_buttons(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;
        ui.horizontal(|ui| {
            for (rpm, label) in [(RPM33, "33"), (RPM45, "45")] {
                let on = (self.base_rpm - rpm).abs() < 0.01;
                let b = egui::Button::new(RichText::new(label).size(16.0).strong().color(if on { Color32::BLACK } else { TEXT }))
                    .fill(if on { LCD_TEXT } else { Color32::from_rgb(40, 44, 52) })
                    .min_size(Vec2::new(54.0, 34.0))
                    .corner_radius(CornerRadius::same(17));
                if ui.add(b).on_hover_text("Platter speed only. The track plays at 1.00x at pitch 0 on both; 45 spins faster for scratching.").clicked() {
                    self.base_rpm = rpm;
                    changed = true;
                }
            }
            let on = self.mode == Mode::Free;
            let b = egui::Button::new(RichText::new("FREE MOTOR  [F]").size(13.0).strong().color(if on { Color32::BLACK } else { TEXT }))
                .fill(if on { ORANGE } else { Color32::from_rgb(40, 44, 52) })
                .min_size(Vec2::new(120.0, 34.0))
                .corner_radius(CornerRadius::same(6));
            if ui.add(b).clicked() {
                self.set_mode(Mode::Free);
            }
        });
        if changed {
            self.push_command();
        }
    }

    // ---------------------------------------------------------------- source

    fn source_row(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("PLATTER").size(10.0).color(TEXT_DIM));
            let text = match &self.src_choice {
                SourceChoice::Wifi => "Wi-Fi platter (ESP32)",
                SourceChoice::Sim => "Simulator (mouse on the disc)",
            };
            egui::ComboBox::from_id_salt("platter_source")
                .selected_text(text)
                .width(330.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.src_choice, SourceChoice::Wifi, "Wi-Fi platter (ESP32)");
                    ui.selectable_value(&mut self.src_choice, SourceChoice::Sim, "Simulator (mouse on the disc)");
                });
            if self.src_choice == SourceChoice::Wifi {
                ui.add(egui::TextEdit::singleline(&mut self.wifi_address).desired_width(165.0));
            }
            let connect = egui::Button::new(RichText::new("Connect").strong().color(Color32::BLACK))
                .fill(LCD_TEXT)
                .corner_radius(CornerRadius::same(4));
            if ui.add(connect).clicked() {
                let src = match &self.src_choice {
                    SourceChoice::Wifi => Source::Wifi(self.wifi_address.trim().to_owned()),
                    SourceChoice::Sim => Source::Sim,
                };
                match self.engine.set_source(src) {
                    Ok(()) => self.status_line = format!("platter source: {}", self.engine.info.source),
                    Err(e) => {
                        self.status_line = format!("connect failed: {e:#}. Back on the simulator.");
                        self.src_choice = SourceChoice::Sim;
                    }
                }
            }
            if !self.engine.info.is_sim && ui.button("Disconnect").clicked() {
                let _ = self.engine.set_source(Source::Sim);
                self.status_line = "Disconnected; motor released. Simulator selected.".into();
                self.src_choice = SourceChoice::Sim;
            }
            ui.label(RichText::new(format!("now: {}", self.engine.info.source)).size(11.0).color(TEXT_DIM));
        });
        if self.src_choice == SourceChoice::Wifi {
            let p = self.engine.platter.snapshot();
            ui.label(RichText::new(format!("Join PlatterDeck Wi-Fi first  |  rejected {}  jitter {:.1} ms  max gap {:.1} ms",
                p.rejected, p.delay_us as f32 / 1000.0, p.max_gap_us as f32 / 1000.0)).size(11.0).color(TEXT_DIM));
        }
    }

    // ---------------------------------------------------------------- display

    fn lcd(&mut self, ui: &mut egui::Ui) {
        let rb = &self.engine.readback;
        let pos = rb.pos();
        let (name, dur, sr) = match &self.engine.track {
            Some(t) => (t.name.clone(), t.duration_s(), t.sample_rate as f64),
            None => ("NO TRACK  -  drop a WAV / MP3 / FLAC on the window".to_string(), 0.0, 1.0),
        };
        let elapsed = (pos / sr).clamp(0.0, dur.max(0.0));
        let remain = (dur - elapsed).max(0.0);
        let rate = rb.rate();

        egui::Frame::new()
            .fill(LCD_BG)
            .stroke(Stroke::new(1.0, Color32::from_rgb(30, 60, 76)))
            .corner_radius(CornerRadius::same(6))
            .inner_margin(Margin::same(10))
            .show(ui, |ui| {
                let w = ui.available_width();
                ui.set_width(w);
                ui.add(egui::Label::new(RichText::new(name).size(14.0).color(LCD_TEXT).strong()).truncate());
                ui.add_space(4.0);
                // Three fixed-width cells so the row can never push the column wider.
                let cell = ((w - 16.0) / 3.0).max(120.0);
                let big = if cell < 170.0 { 24.0 } else { 30.0 };
                ui.horizontal(|ui| {
                    ui.allocate_ui_with_layout(Vec2::new(cell, 50.0), egui::Layout::top_down(egui::Align::Min), |ui| {
                        ui.label(RichText::new("TIME").size(10.0).color(LCD_DIM));
                        ui.label(RichText::new(fmt_time(elapsed)).monospace().size(big).color(LCD_TEXT));
                    });
                    ui.allocate_ui_with_layout(Vec2::new(cell, 50.0), egui::Layout::top_down(egui::Align::Min), |ui| {
                        ui.label(RichText::new("REMAIN").size(10.0).color(LCD_DIM));
                        ui.label(RichText::new(format!("-{}", fmt_time(remain))).monospace().size(big).color(LCD_TEXT));
                    });
                    ui.allocate_ui_with_layout(Vec2::new(cell, 50.0), egui::Layout::top_down(egui::Align::Min), |ui| {
                        ui.label(RichText::new("SPEED  x nominal").size(10.0).color(LCD_DIM));
                        ui.label(RichText::new(format!("x{:+.2}", rate)).monospace().size(big * 0.8).color(LCD_TEXT));
                        ui.label(
                            RichText::new(format!("pitch {:+.2}%  {} rpm", self.pitch_pct, if self.base_rpm > 40.0 { "45" } else { "33" }))
                                .monospace()
                                .size(11.0)
                                .color(LCD_DIM),
                        );
                    });
                });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label(RichText::new("LEVEL").size(10.0).color(LCD_DIM));
                    let mw = (ui.available_width() - 8.0).max(80.0);
                    let (_, painter) = ui.allocate_painter(Vec2::new(mw, 26.0), Sense::hover());
                    let r = painter.clip_rect();
                    draw_meter(&painter, Rect::from_min_size(r.min, Vec2::new(mw, 11.0)), self.meter_l);
                    draw_meter(&painter, Rect::from_min_size(r.min + Vec2::new(0.0, 15.0), Vec2::new(mw, 11.0)), self.meter_r);
                });
                ui.add_space(2.0);
                ui.add(egui::Label::new(RichText::new(&self.status_line).size(11.0).color(LCD_DIM)).truncate());
            });
    }

    fn transport(&mut self, ui: &mut egui::Ui) {
        let h = 58.0;
        ui.horizontal(|ui| {
            let spinning = self.mode == Mode::Spin && !self.braking;
            let b = egui::Button::new(
                RichText::new(if spinning { "PAUSE\nBRAKE" } else { "PLAY\nSPIN" }).size(15.0).strong().color(Color32::BLACK),
            )
            .fill(if spinning { GREEN } else { GREEN.gamma_multiply(0.45) })
            .min_size(Vec2::new(120.0, h))
            .corner_radius(CornerRadius::same(8));
            if ui.add(b).clicked() {
                self.toggle_play();
            }

            let b = egui::Button::new(RichText::new("CUE").size(18.0).strong().color(Color32::BLACK))
                .fill(if self.cues[0].is_some() { ORANGE } else { ORANGE.gamma_multiply(0.45) })
                .min_size(Vec2::new(100.0, h))
                .corner_radius(CornerRadius::same(8));
            let shift = ui.input(|i| i.modifiers.shift);
            let r = ui.add(b).on_hover_text("click: jump to cue 1 (sets it when empty)\nshift+click: set cue 1\nright-click: clear");
            if r.clicked() {
                self.cue_press(0, shift);
            }
            if r.secondary_clicked() {
                self.cues[0] = None;
            }

            let braking = self.mode == Mode::Brake;
            let b = egui::Button::new(RichText::new("STOP\nE-STOP").size(13.0).strong().color(Color32::WHITE))
                .fill(if self.mode == Mode::Estop { RED } else { RED.gamma_multiply(0.4) })
                .min_size(Vec2::new(90.0, h))
                .corner_radius(CornerRadius::same(8));
            let r = ui.add(b).on_hover_text("click: brake to 0\nshift+click: E-STOP, driver off");
            if r.clicked() {
                self.set_mode(if shift || braking { Mode::Estop } else { Mode::Brake });
            }
        });
    }

    fn hot_cues(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("HOT CUES   click: jump / set when empty   shift+click: set   right-click: clear   keys 1-4").size(10.0).color(TEXT_DIM));
        let shift = ui.input(|i| i.modifiers.shift);
        let sr = self.engine.track.as_ref().map(|t| t.sample_rate as f64).unwrap_or(1.0);
        ui.horizontal(|ui| {
            for i in 0..4 {
                let col = PAD_COLORS[i];
                let set = self.cues[i].is_some();
                let label = match self.cues[i] {
                    Some(s) => format!("{}\n{}", i + 1, fmt_time(s / sr)),
                    None => format!("{}\n--:--", i + 1),
                };
                let b = egui::Button::new(RichText::new(label).monospace().size(13.0).strong().color(if set { Color32::BLACK } else { TEXT_DIM }))
                    .fill(if set { col } else { col.gamma_multiply(0.18) })
                    .stroke(Stroke::new(1.0, col.gamma_multiply(0.7)))
                    .min_size(Vec2::new(96.0, 52.0))
                    .corner_radius(CornerRadius::same(6));
                let r = ui.add(b);
                if r.clicked() {
                    self.cue_press(i, shift);
                }
                if r.secondary_clicked() {
                    self.cues[i] = None;
                }
            }
        });
    }

    fn deck_knobs(&mut self, ui: &mut egui::Ui) {
        egui::Frame::new()
            .fill(PANEL)
            .stroke(Stroke::new(1.0, PANEL_EDGE))
            .corner_radius(CornerRadius::same(6))
            .inner_margin(Margin::same(8))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.label(RichText::new("DECK   drag a knob up / down, double-click resets").size(10.0).color(TEXT_DIM));
                ui.horizontal(|ui| {
                    let text = format!("{:.2}", self.gain);
                    if knob(ui, &mut self.gain, 0.0, 1.5, 1.0, "GAIN", text, "output level") {
                        self.engine.set_gain(self.gain);
                    }
                    let text = fmt_ramp(self.start_s);
                    knob(ui, &mut self.start_s, 0.0, 3.0, 0.0, "START", text, "platter wind-up time after PLAY");
                    let text = fmt_ramp(self.brake_s);
                    knob(ui, &mut self.brake_s, 0.0, 3.0, 0.0, "BRAKE", text, "platter wind-down time after PAUSE");
                });
            });
    }

    fn motor_box(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;
        egui::Frame::new()
            .fill(PANEL)
            .stroke(Stroke::new(1.0, PANEL_EDGE))
            .corner_radius(CornerRadius::same(6))
            .inner_margin(Margin::same(8))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.label(RichText::new("MOTOR").size(10.0).color(TEXT_DIM));
                changed |= ui
                    .add(egui::Slider::new(&mut self.torque_v, 0.1..=3.0).suffix(" V").text("torque cap"))
                    .on_hover_text("SimpleFOC voltage_limit: how hard the platter fights your hand")
                    .changed();
                if ui
                    .add(egui::Slider::new(&mut self.lead_ms, 0.0..=30.0).suffix(" ms").text("extra lead"))
                    .on_hover_text("added to the measured output latency when aiming the platter timeline; raise if scratches feel late")
                    .changed()
                {
                    self.engine.set_extra_lead_us(self.lead_ms as f64 * 1000.0);
                }
            });
        if changed {
            self.push_command();
        }
    }

    fn track_row(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("TRACK").strong());
            if ui.button("Open track...").clicked() {
                if let Some(path) = crate::sys::choose_track() { self.load(path); }
            }
            let mut selected = None;
            egui::ComboBox::from_id_salt("recent_tracks")
                .selected_text("Choose previous track")
                .width(205.0)
                .show_ui(ui, |ui| {
                    for path in &self.recent_tracks {
                        let name = path.file_name().unwrap_or_default().to_string_lossy();
                        if ui.selectable_label(false, name).on_hover_text(path.display().to_string()).clicked() {
                            selected = Some(path.clone());
                        }
                    }
                });
            if let Some(path) = selected { self.load(path); }
            if ui.add_enabled(self.engine.track.is_some() || !self.path_text.is_empty(), egui::Button::new("Remove track")).clicked() {
                self.engine.eject(); self.cues = [None; 4]; self.path_text.clear();
                self.status_line = "No track loaded. Open a track or choose a previous one.".into();
            }
            if ui.add_enabled(self.engine.track.is_some(), egui::Button::new("Restart track")).clicked() {
                self.engine.jump(0.0);
            }
            let name = self.engine.track.as_ref().map(|t| t.name.as_str()).unwrap_or("No track loaded");
            ui.add(egui::Label::new(RichText::new(name).color(LCD_TEXT)).truncate());
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("PLAYBACK");
            let changed = ui.selectable_value(&mut self.steady_playback, true, "STEADY").clicked()
                | ui.selectable_value(&mut self.steady_playback, false, "VINYL").clicked();
            if changed { self.engine.set_steady_playback(self.steady_playback); }
            ui.label(RichText::new(if self.steady_playback {
                "Tempo from the audio clock and the pitch fader; pitch 0 = original speed at 33 or 45. Select VINYL to scratch."
            } else {
                "Music follows the physical platter; nominal platter speed = 1.00x. Motor speed changes affect pitch."
            }).small().color(TEXT_DIM));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.menu_button(RichText::new("HELP \u{25BE}").strong().color(LCD_TEXT), help_menu);
            });
        });
    }

    fn pitch_fader(&mut self, ui: &mut egui::Ui) {
        egui::Frame::new()
            .fill(PANEL)
            .stroke(Stroke::new(1.0, PANEL_EDGE))
            .corner_radius(CornerRadius::same(6))
            .inner_margin(Margin::same(8))
            .show(ui, |ui| {
                ui.vertical_centered(|ui| {
                    ui.label(RichText::new("PITCH").size(10.0).color(TEXT_DIM));
                    let range = self.pitch_range;
                    ui.label(RichText::new(format!("-{range:.0}")).monospace().size(11.0).color(TEXT_DIM));
                    // Hardware convention: + at the bottom. Bind the slider to
                    // the negated value so the top is slow and the bottom fast.
                    let mut v = -self.pitch_pct;
                    ui.spacing_mut().slider_width = 215.0;
                    let r = ui.add(egui::Slider::new(&mut v, -range..=range).vertical().show_value(false).step_by(0.05));
                    // Centre detent marker.
                    let rect = r.rect;
                    let y = rect.center().y;
                    ui.painter().line_segment(
                        [Pos2::new(rect.left() - 4.0, y), Pos2::new(rect.right() + 4.0, y)],
                        Stroke::new(1.0, TEXT_DIM),
                    );
                    if r.changed() {
                        self.pitch_pct = -v;
                        self.push_command();
                    }
                    ui.label(RichText::new(format!("+{range:.0}")).monospace().size(11.0).color(TEXT_DIM));
                    ui.label(RichText::new(format!("{:+.2}%", self.pitch_pct)).monospace().size(14.0).color(LCD_TEXT));
                    ui.horizontal(|ui| {
                        if ui.small_button("0").on_hover_text("pitch reset").clicked() {
                            self.pitch_pct = 0.0;
                            self.push_command();
                        }
                        // Pitch range switch, as on a turntable: 8 / 16 / 50 %.
                        if ui.small_button(format!("\u{00B1}{range:.0}")).on_hover_text("pitch range: 8 / 16 / 50 %").clicked() {
                            self.pitch_range = if range < 12.0 { 16.0 } else if range < 30.0 { 50.0 } else { 8.0 };
                            self.pitch_pct = self.pitch_pct.clamp(-self.pitch_range, self.pitch_range);
                            self.push_command();
                        }
                    });
                    // Pitch bend: hold to nudge the beat, release to return.
                    ui.horizontal(|ui| {
                        let size = Vec2::new(32.0, 22.0);
                        let minus = ui.add(egui::Button::new(RichText::new("\u{2212}").strong()).min_size(size)).on_hover_text("hold: bend -4 %");
                        let plus = ui.add(egui::Button::new(RichText::new("+").strong()).min_size(size)).on_hover_text("hold: bend +4 %");
                        let bend = if minus.is_pointer_button_down_on() {
                            -4.0
                        } else if plus.is_pointer_button_down_on() {
                            4.0
                        } else {
                            0.0
                        };
                        if bend != self.bend_pct {
                            self.bend_pct = bend;
                            self.push_command();
                        }
                    });
                    ui.label(RichText::new(format!("{:.2} rpm", self.target_rpm())).monospace().size(11.0).color(TEXT_DIM));
                });
            });
    }

    // ---------------------------------------------------------------- waveforms

    fn waveforms(&mut self, ui: &mut egui::Ui) {
        let w = ui.available_width();
        let zoom_h = 110.0;
        let over_h = 64.0;
        let Some(t) = self.engine.track.clone() else {
            let (_, painter) = ui.allocate_painter(Vec2::new(w, zoom_h + over_h + 6.0), Sense::hover());
            let rect = painter.clip_rect();
            painter.rect_filled(rect, CornerRadius::same(4), Color32::from_rgb(10, 11, 13));
            painter.text(rect.center(), Align2::CENTER_CENTER, "drop a WAV / MP3 / FLAC here", FontId::proportional(15.0), TEXT_DIM);
            return;
        };
        let pos = self.engine.readback.pos();
        let sr = t.sample_rate as f64;
        let frames = t.frames.max(1) as f64;

        // Zoomed: +-4 s around the playhead, drawn straight from the samples.
        {
            let (resp, painter) = ui.allocate_painter(Vec2::new(w, zoom_h), Sense::click());
            let rect = resp.rect;
            painter.rect_filled(rect, CornerRadius::same(4), Color32::from_rgb(10, 11, 13));
            let span = 8.0 * sr;
            let start = pos - span / 2.0;
            let px = rect.width().max(1.0) as usize;
            let per_px = span / px as f64;
            let stride = ((per_px / 24.0) as usize).max(1);
            let mid = rect.center().y;
            for x in 0..px {
                let s0 = start + x as f64 * per_px;
                let s1 = s0 + per_px;
                if s1 < 0.0 || s0 >= frames {
                    continue;
                }
                let a = s0.max(0.0) as usize;
                let b = (s1.min(frames) as usize).max(a + 1);
                let mut pk = 0f32;
                let mut i = a;
                while i < b {
                    pk = pk.max(t.samples[i * 2].abs()).max(t.samples[i * 2 + 1].abs());
                    i += stride;
                }
                let y = pk * (zoom_h * 0.48);
                let xx = rect.left() + x as f32 + 0.5;
                let col = if (s0 as f64) < pos { ORANGE.gamma_multiply(0.75) } else { wave_color(pk) };
                painter.line_segment([Pos2::new(xx, mid - y), Pos2::new(xx, mid + y)], Stroke::new(1.0, col));
            }
            // Second markers.
            let first_sec = (start / sr).ceil() as i64;
            let last_sec = ((start + span) / sr).floor() as i64;
            for s in first_sec..=last_sec {
                if s < 0 {
                    continue;
                }
                let x = rect.left() + ((s as f64 * sr - start) / span) as f32 * rect.width();
                painter.line_segment([Pos2::new(x, rect.bottom() - 6.0), Pos2::new(x, rect.bottom())], Stroke::new(1.0, TEXT_DIM));
            }
            for (i, c) in self.cues.iter().enumerate() {
                if let Some(s) = c {
                    if *s >= start && *s <= start + span {
                        let x = rect.left() + ((s - start) / span) as f32 * rect.width();
                        painter.line_segment([Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())], Stroke::new(2.0, PAD_COLORS[i]));
                    }
                }
            }
            let xc = rect.center().x;
            painter.line_segment([Pos2::new(xc, rect.top()), Pos2::new(xc, rect.bottom())], Stroke::new(2.0, PLAYHEAD));
            if resp.clicked() {
                if let Some(p) = resp.interact_pointer_pos() {
                    let frac = ((p.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64;
                    self.engine.jump((start + frac * span).clamp(0.0, frames));
                }
            }
        }
        ui.add_space(6.0);
        // Overview.
        {
            let (resp, painter) = ui.allocate_painter(Vec2::new(w, over_h), Sense::click());
            let rect = resp.rect;
            painter.rect_filled(rect, CornerRadius::same(4), Color32::from_rgb(10, 11, 13));
            let bins = t.overview.len().max(1);
            let mid = rect.center().y;
            let px = rect.width().max(1.0) as usize;
            let played_px = (pos / frames * px as f64) as usize;
            for x in 0..px {
                let b0 = x * bins / px;
                let b1 = ((x + 1) * bins / px).max(b0 + 1).min(bins);
                let pk = t.overview[b0..b1].iter().cloned().fold(0f32, f32::max);
                let y = pk * (over_h * 0.47);
                let xx = rect.left() + x as f32 + 0.5;
                let col = if x < played_px { ORANGE.gamma_multiply(0.6) } else { wave_color(pk) };
                painter.line_segment([Pos2::new(xx, mid - y), Pos2::new(xx, mid + y)], Stroke::new(1.0, col));
            }
            for (i, c) in self.cues.iter().enumerate() {
                if let Some(s) = c {
                    let x = rect.left() + (s / frames) as f32 * rect.width();
                    painter.line_segment([Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())], Stroke::new(1.5, PAD_COLORS[i]));
                    painter.text(Pos2::new(x + 3.0, rect.top() + 1.0), Align2::LEFT_TOP, format!("{}", i + 1), FontId::monospace(10.0), PAD_COLORS[i]);
                }
            }
            let xh = rect.left() + (pos / frames).clamp(0.0, 1.0) as f32 * rect.width();
            painter.line_segment([Pos2::new(xh, rect.top()), Pos2::new(xh, rect.bottom())], Stroke::new(2.0, PLAYHEAD));
            painter.text(
                Pos2::new(rect.right() - 6.0, rect.bottom() - 2.0),
                Align2::RIGHT_BOTTOM,
                format!("{} / {}", fmt_time(pos / sr), fmt_time(t.duration_s())),
                FontId::monospace(11.0),
                TEXT_DIM,
            );
            if resp.clicked() {
                if let Some(p) = resp.interact_pointer_pos() {
                    let frac = ((p.x - rect.left()) / rect.width()).clamp(0.0, 1.0);
                    self.engine.jump(frac as f64 * frames);
                }
            }
        }
    }
}

impl eframe::App for App {
    // egui 0.36: the app draws into a root `Ui`; panels are shown inside it.
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Reflect link-enforced FREE/E-STOP so a later pitch change cannot
        // accidentally restore an old SPIN after reconnection.
        self.mode = self.engine.command().mode;
        self.update_ramp();
        let ctx = root.ctx().clone();
        let ctx = &ctx;
        for ev in self.engine.poll() {
            match ev {
                EngineEvent::TrackLoaded(t) => {
                    self.status_line = format!("loaded {} ({} Hz, {})", t.name, t.sample_rate, fmt_time(t.duration_s()));
                    self.cues = [None; 4];
                }
                EngineEvent::LoadFailed(e) => self.status_line = format!("load failed: {e}"),
            }
        }
        let dropped: Vec<PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).collect());
        if let Some(p) = dropped.into_iter().next() {
            self.load(p);
        }
        self.handle_keys(ctx);

        let p = self.engine.platter.snapshot();
        self.update_meters(&p);

        egui::Panel::top("top").show(root, |ui| self.top_bar(ui, &p));
        egui::Panel::top("tracks").show(root, |ui| self.track_row(ui));
        egui::Panel::bottom("wave")
            .frame(egui::Frame::new().fill(BG).inner_margin(Margin::same(8)))
            .show(root, |ui| self.waveforms(ui));
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(BG).inner_margin(Margin::same(10)))
            .show(root, |ui| {
                ui.horizontal_top(|ui| {
                    // Platter column.
                    let platter_w = (ui.available_width() * 0.40).clamp(280.0, 480.0);
                    ui.allocate_ui_with_layout(
                        Vec2::new(platter_w, ui.available_height()),
                        egui::Layout::top_down(egui::Align::Center),
                        |ui| {
                            self.platter(ui, &p);
                            ui.add_space(6.0);
                            self.speed_buttons(ui);
                        },
                    );
                    ui.add_space(8.0);
                    // Pitch fader column.
                    ui.allocate_ui_with_layout(
                        Vec2::new(92.0, ui.available_height()),
                        egui::Layout::top_down(egui::Align::Center),
                        |ui| self.pitch_fader(ui),
                    );
                    ui.add_space(8.0);
                    // Display + transport column. Pin its width so wide
                    // content clips instead of pushing past the window.
                    let col_w = ui.available_width().max(360.0);
                    ui.allocate_ui_with_layout(
                        Vec2::new(col_w, ui.available_height()),
                        egui::Layout::top_down(egui::Align::Min),
                        |ui| {
                        ui.set_max_width(col_w);
                        egui::ScrollArea::vertical().id_salt("deck_controls").show(ui, |ui| {
                        self.source_row(ui);
                        ui.add_space(6.0);
                        self.lcd(ui);
                        ui.add_space(8.0);
                        self.transport(ui);
                        ui.add_space(8.0);
                        self.hot_cues(ui);
                        ui.add_space(8.0);
                        self.deck_knobs(ui);
                        ui.add_space(8.0);
                        self.motor_box(ui);
                        });
                        },
                    );
                });
            });
        ctx.request_repaint_after(Duration::from_millis(16));
    }
}

/// Small rotary knob: drag up or down to turn, double-click for the default.
/// 270 degree sweep from 7 o'clock to 5 o'clock. Returns true when changed.
#[allow(clippy::too_many_arguments)]
fn knob(ui: &mut egui::Ui, value: &mut f32, min: f32, max: f32, default: f32, label: &str, text: String, hint: &str) -> bool {
    let mut changed = false;
    ui.vertical(|ui| {
        ui.set_width(66.0);
        ui.vertical_centered(|ui| {
            ui.label(RichText::new(label).size(10.0).color(TEXT_DIM));
            let (rect, resp) = ui.allocate_exact_size(Vec2::splat(42.0), Sense::click_and_drag());
            let resp = resp.on_hover_text(hint);
            if resp.dragged() {
                let v = (*value - resp.drag_delta().y * (max - min) / 140.0).clamp(min, max);
                if v != *value {
                    *value = v;
                    changed = true;
                }
            }
            if resp.double_clicked() && *value != default {
                *value = default;
                changed = true;
            }
            let c = rect.center();
            let r = 17.0;
            let live = resp.hovered() || resp.dragged();
            let p = ui.painter().clone();
            p.circle_filled(c, r, PANEL_EDGE);
            p.circle_stroke(c, r, Stroke::new(1.0, if live { LCD_TEXT } else { TEXT_DIM }));
            let dir = |frac: f32| {
                let a = (135.0 + 270.0 * frac).to_radians();
                Vec2::new(a.cos(), a.sin())
            };
            for stop in [0.0f32, 0.5, 1.0] {
                let d = dir(stop);
                p.line_segment([c + d * (r + 2.0), c + d * (r + 5.0)], Stroke::new(1.0, TEXT_DIM));
            }
            let d = dir(((*value - min) / (max - min)).clamp(0.0, 1.0));
            p.line_segment([c + d * 5.0, c + d * (r - 2.0)], Stroke::new(2.5, ORANGE));
            ui.label(RichText::new(text).monospace().size(11.0).color(LCD_TEXT));
        });
    });
    changed
}

fn fmt_ramp(seconds: f32) -> String {
    if seconds < 0.05 { "fast".into() } else { format!("{seconds:.1} s") }
}

fn wave_color(peak: f32) -> Color32 {
    let t = peak.clamp(0.0, 1.0);
    let lerp = |a: u8, b: u8| (a as f32 + (b as f32 - a as f32) * t) as u8;
    Color32::from_rgb(lerp(WAVE_LO.r(), WAVE_HI.r()), lerp(WAVE_LO.g(), WAVE_HI.g()), lerp(WAVE_LO.b(), WAVE_HI.b()))
}

fn draw_meter(painter: &egui::Painter, rect: Rect, level: f32) {
    painter.rect_filled(rect, CornerRadius::same(2), Color32::from_rgb(12, 30, 40));
    // dB scale: -40 dB at the left edge, 0 dB at the right.
    let db = 20.0 * level.max(1e-5).log10();
    let frac = ((db + 40.0) / 40.0).clamp(0.0, 1.0);
    let segs = 20;
    let gap = 2.0;
    let seg_w = (rect.width() - gap * (segs as f32 - 1.0)) / segs as f32;
    let lit = (frac * segs as f32).round() as usize;
    for i in 0..segs {
        let x = rect.left() + i as f32 * (seg_w + gap);
        let col = if i >= 18 { RED } else if i >= 14 { ORANGE } else { GREEN };
        let col = if i < lit { col } else { col.gamma_multiply(0.15) };
        painter.rect_filled(Rect::from_min_size(Pos2::new(x, rect.top()), Vec2::new(seg_w, rect.height())), CornerRadius::same(1), col);
    }
    painter.rect_stroke(rect, CornerRadius::same(2), Stroke::new(0.5, Color32::from_rgb(30, 60, 76)), StrokeKind::Inside);
}

fn fmt_time(s: f64) -> String {
    let s = s.max(0.0);
    let m = (s / 60.0).floor();
    format!("{:02}:{:04.1}", m as u32, s - m * 60.0)
}
