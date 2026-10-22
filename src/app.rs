use crate::{
    controller::{Action, Controller, Snapshot},
    gcode::Point,
};
use egui::{Color32, Pos2, RichText, Stroke, Vec2};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const CYAN: Color32 = Color32::from_rgb(75, 211, 223);
const GREEN: Color32 = Color32::from_rgb(110, 220, 160);
const ORANGE: Color32 = Color32::from_rgb(255, 185, 90);
const RED: Color32 = Color32::from_rgb(255, 105, 105);

#[derive(Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub address: String,
    pub http: String,
    pub pierce: f64,
    pub jog_speed: f64,
    pub last_dir: PathBuf,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            address: "192.168.0.1:23".into(),
            http: "http://192.168.0.1".into(),
            pierce: 0.5,
            jog_speed: 600.0,
            last_dir: std::env::var_os("HOME")
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from(".")),
        }
    }
}
impl Settings {
    fn path() -> PathBuf {
        std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".config")
            })
            .join("plasma-wifi/settings.json")
    }
    pub fn load() -> Self {
        std::fs::read(Self::path())
            .ok()
            .and_then(|s| serde_json::from_slice(&s).ok())
            .unwrap_or_default()
    }
    fn save(&self) {
        let path = Self::path();
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        if let Ok(data) = serde_json::to_vec_pretty(self) {
            let tmp = path.with_extension("tmp");
            if std::fs::write(&tmp, data).is_ok() {
                let _ = std::fs::rename(tmp, path);
            }
        }
    }
}

pub struct App {
    pub controller: Controller,
    settings: Settings,
    snapshot: Snapshot,
    dro: [String; 2],
    dro_edit: [bool; 2],
    show_original: bool,
    auto_scroll: bool,
    last_active: Option<usize>,
    scroll_to_active: bool,
    chooser: bool,
    chooser_path: String,
    settings_open: bool,
    log_open: bool,
    error: Option<String>,
    was_jogging: bool,
    zoom: f32,
    pan: Vec2,
    demo: bool,
    applied_pierce: f64,
}
impl App {
    pub fn new(settings: Settings, demo: bool) -> Self {
        Self {
            controller: Controller::start(settings.address.clone(), settings.http.clone()),
            snapshot: Snapshot::default(),
            applied_pierce: settings.pierce,
            settings,
            dro: ["0.000".into(), "0.000".into()],
            dro_edit: [false; 2],
            show_original: false,
            auto_scroll: true,
            last_active: None,
            scroll_to_active: false,
            chooser: false,
            chooser_path: String::new(),
            settings_open: false,
            log_open: false,
            error: None,
            was_jogging: false,
            zoom: 1.0,
            pan: Vec2::ZERO,
            demo,
        }
    }
    pub fn open(&mut self, path: PathBuf) {
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                if let Some(parent) = path.parent() {
                    self.settings.last_dir = parent.to_owned();
                    if !self.demo {
                        self.settings.save();
                    }
                }
                self.controller.send(Action::Load {
                    name: path
                        .file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into(),
                    text,
                    pierce: self.settings.pierce,
                });
                self.zoom = 1.0;
                self.pan = Vec2::ZERO;
                self.chooser = false;
                self.error = None;
            }
            Err(e) => self.error = Some(format!("{}: {e}", path.display())),
        }
    }
    pub fn load_text(&self, name: &str, text: &str) {
        self.controller.send(Action::Load {
            name: name.into(),
            text: text.into(),
            pierce: self.settings.pierce,
        });
    }
    pub fn ui(&mut self, root: &mut egui::Ui) {
        self.snapshot = self.controller.snapshot();
        let ctx = root.ctx().clone();
        let mut visuals = egui::Visuals::dark();
        visuals.panel_fill = Color32::from_rgb(20, 26, 35);
        visuals.window_fill = Color32::from_rgb(26, 34, 45);
        visuals.selection.bg_fill = Color32::from_rgb(25, 100, 115);
        ctx.set_visuals(visuals);
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::O)) {
            self.open_chooser();
        }
        egui::Panel::top("header").show_inside(root, |ui| {
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                ui.heading(RichText::new("PLASMA / WIFI").color(CYAN));
                ui.separator();
                let (text, color) = self.state_label();
                ui.label(RichText::new(text).size(24.0).strong().color(color));
                if self.demo {
                    ui.label(RichText::new("SIMULATED CONTROLLER").color(ORANGE));
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Settings").clicked() {
                        self.settings_open = true;
                    }
                    if ui.button("Log").clicked() {
                        self.log_open = !self.log_open;
                    }
                    if ui
                        .add_enabled(
                            !self.snapshot.job && !self.snapshot.transferring,
                            egui::Button::new("Open G-code…  Ctrl+O"),
                        )
                        .clicked()
                    {
                        self.open_chooser();
                    }
                });
            });
            ui.horizontal(|ui| {
                ui.label(if self.snapshot.name.is_empty() {
                    "No file selected"
                } else {
                    &self.snapshot.name
                });
                ui.label(
                    RichText::new(&self.snapshot.upload).color(if self.snapshot.ready {
                        GREEN
                    } else {
                        ORANGE
                    }),
                );
            });
            ui.add_space(5.0);
        });
        egui::Panel::bottom("footer").show_inside(root, |ui| {
            ui.horizontal(|ui| {
                if let Some(program) = &self.snapshot.program {
                    ui.small(format!(
                        "Original: {} lines · {} bytes    Uploaded: {} lines · {} bytes",
                        program.source_lines.len(),
                        program.original.len(),
                        program.blocks.len(),
                        program.uploaded.len()
                    ));
                }
                if let Some(free) = &self.snapshot.free {
                    ui.separator();
                    ui.small(format!("LocalFS: {free}"));
                }
            });
            let error = self.error.as_ref().or(self.snapshot.error.as_ref());
            if let Some(error) = error {
                ui.colored_label(RED, error);
            }
        });
        egui::Panel::left("controls")
            .resizable(false)
            .exact_size(268.0)
            .show_inside(root, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.controls(ui));
            });
        egui::CentralPanel::default().show_inside(root, |ui| {
            let preview_height = (ui.available_height() * 0.52).max(140.0);
            self.preview(ui, preview_height);
            ui.add_space(5.0);
            self.code_view(ui);
        });
        self.dialogs(root);
        self.jog_input(&ctx);
    }
    fn state_label(&self) -> (String, Color32) {
        let state = &self.snapshot.status.state;
        if !self.snapshot.connected {
            return ("DISCONNECTED".into(), RED);
        }
        if state.starts_with("Hold") {
            return ("PAUSED".into(), ORANGE);
        }
        match state.as_str() {
            "Idle" => (
                if self.snapshot.job { "RUNNING" } else { "IDLE" }.into(),
                GREEN,
            ),
            "Run" => ("RUNNING".into(), CYAN),
            "Jog" => ("JOGGING".into(), CYAN),
            _ => (
                state.to_uppercase(),
                if state.contains("Alarm") { RED } else { ORANGE },
            ),
        }
    }
    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.0);
        ui.label(RichText::new("WORK POSITION  /  mm").color(CYAN).strong());
        let can_set = self.snapshot.connected
            && self.snapshot.initialized
            && self.snapshot.status.idle()
            && !self.snapshot.job
            && self.snapshot.status.position_known;
        let can_set = can_set && !self.snapshot.transferring;
        let position = self.snapshot.status.wpos;
        for (i, axis, value) in [(0, 'X', position.x), (1, 'Y', position.y)] {
            if !self.dro_edit[i] {
                self.dro[i] = format!("{value:.3}");
            }
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(
                        can_set,
                        egui::Button::new(format!("{axis}0")).min_size(Vec2::new(40.0, 36.0)),
                    )
                    .on_hover_text("Set this work coordinate to zero without moving")
                    .clicked()
                {
                    self.controller
                        .send(Action::SetCoordinate { axis, value: 0.0 });
                }
                let response = ui.add_enabled(
                    can_set,
                    egui::TextEdit::singleline(&mut self.dro[i])
                        .font(egui::TextStyle::Heading)
                        .desired_width(126.0),
                );
                self.dro_edit[i] = response.has_focus();
                if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    match self.dro[i].parse::<f64>() {
                        Ok(value) if value.is_finite() => {
                            self.controller.send(Action::SetCoordinate { axis, value });
                            self.error = None;
                        }
                        _ => self.error = Some("Enter a finite coordinate in millimetres".into()),
                    }
                }
                if ui
                    .add_enabled(
                        can_set,
                        egui::Button::new("/2").min_size(Vec2::new(34.0, 36.0)),
                    )
                    .on_hover_text("Halve the displayed work coordinate without moving")
                    .clicked()
                {
                    self.controller.send(Action::SetCoordinate {
                        axis,
                        value: value / 2.0,
                    });
                }
            });
        }
        if !self.snapshot.status.position_known {
            ui.small("Waiting for work offset / position…");
        }
        ui.small("Edit a number, then press Enter. No axis motion.");
        ui.add_space(9.0);
        ui.separator();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    self.snapshot.can_run(),
                    egui::Button::new(RichText::new("RUN").strong().color(GREEN))
                        .min_size(Vec2::new(75.0, 38.0)),
                )
                .clicked()
            {
                self.controller.send(Action::Run);
            }
            let paused = self.snapshot.status.state.starts_with("Hold");
            let resume_ready = self.snapshot.phase == "Paused · torch off";
            if ui
                .add_enabled(
                    self.snapshot.job && self.snapshot.connected && (!paused || resume_ready),
                    egui::Button::new(if paused { "Resume" } else { "Pause" })
                        .min_size(Vec2::new(75.0, 38.0)),
                )
                .clicked()
            {
                self.controller.send(if paused {
                    Action::Resume
                } else {
                    Action::Pause
                });
            }
            if ui
                .add_enabled(
                    self.snapshot.connected,
                    egui::Button::new(RichText::new("STOP").color(RED))
                        .min_size(Vec2::new(70.0, 38.0)),
                )
                .clicked()
            {
                self.controller.send(Action::Stop);
            }
        });
        if self.snapshot.status.state.contains("Alarm") && ui.button("Unlock alarm").clicked() {
            self.controller.send(Action::Unlock);
        }
        ui.label(RichText::new(&self.snapshot.phase).color(ORANGE));
        ui.add(egui::ProgressBar::new(self.snapshot.progress as f32).show_percentage());
        egui::Grid::new("time")
            .num_columns(2)
            .spacing([20.0, 5.0])
            .show(ui, |ui| {
                ui.label("Elapsed");
                ui.label(RichText::new(duration(self.snapshot.elapsed)).size(23.0));
                ui.end_row();
                ui.label("Remaining ≈");
                ui.label(RichText::new(duration(self.snapshot.remaining)).size(23.0));
                ui.end_row();
            });
        ui.small(format!(
            "Motion correction: {:.2}×",
            self.snapshot.coefficient
        ));
        if self.snapshot.timing_uncertain {
            ui.colored_label(ORANGE, "Timing uncertain after connection loss");
        }
        if self.snapshot.connected && !self.snapshot.line_numbers {
            ui.colored_label(
                ORANGE,
                "Enable use_line_numbers in config.yaml for active-line tracking.",
            );
        }
        ui.add_space(8.0);
        ui.separator();
        ui.label(RichText::new("FEED OVERRIDE").color(CYAN).strong());
        ui.label(
            RichText::new(format!(
                "{:.0}%",
                self.snapshot.status.feed_override * 100.0
            ))
            .size(28.0),
        );
        ui.horizontal(|ui| {
            for (label, byte) in [
                ("−10", 0x92),
                ("−1", 0x94),
                ("100%", 0x90),
                ("+1", 0x93),
                ("+10", 0x91),
            ] {
                if ui
                    .add_enabled(self.snapshot.connected, egui::Button::new(label))
                    .clicked()
                {
                    self.controller.send(Action::Feed(byte));
                }
            }
        });
        ui.small(format!(
            "Actual feed: {:.0} mm/min",
            self.snapshot.status.feed
        ));
        ui.add_space(10.0);
        ui.separator();
        ui.label(RichText::new("JOG  /  hold to move").color(CYAN).strong());
        ui.horizontal(|ui| {
            ui.label("Speed");
            if ui
                .add(
                    egui::DragValue::new(&mut self.settings.jog_speed)
                        .range(1.0..=3000.0)
                        .speed(10.0)
                        .suffix(" mm/min"),
                )
                .changed()
                && !self.demo
            {
                self.settings.save();
            }
        });
        let mut pointer_jog = None;
        ui.add_enabled_ui(
            self.snapshot.can_jog() && !self.snapshot.status.torch,
            |ui| {
                egui::Grid::new("jog_pad")
                    .spacing([5.0, 5.0])
                    .show(ui, |ui| {
                        for row in [
                            [("↖", -1.0, 1.0), ("Y+", 0.0, 1.0), ("↗", 1.0, 1.0)],
                            [("X−", -1.0, 0.0), ("•", 0.0, 0.0), ("X+", 1.0, 0.0)],
                            [("↙", -1.0, -1.0), ("Y−", 0.0, -1.0), ("↘", 1.0, -1.0)],
                        ] {
                            for (label, x, y) in row {
                                let response = ui.add_sized([72.0, 38.0], egui::Button::new(label));
                                if response.is_pointer_button_down_on() && (x != 0.0 || y != 0.0) {
                                    pointer_jog = Some((x, y));
                                }
                            }
                            ui.end_row();
                        }
                    });
            },
        );
        ui.ctx()
            .data_mut(|d| d.insert_temp(egui::Id::new("pointer_jog"), pointer_jog));
        ui.small("Arrow keys jog when no text field is focused.");
        ui.small(format!(
            "Torch: {}",
            if self.snapshot.status.torch {
                "ON"
            } else {
                "off"
            }
        ));
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Pierce delay");
            ui.label(format!("{:.3} s", self.applied_pierce));
        });
    }
    fn jog_input(&mut self, ctx: &egui::Context) {
        let pointer = ctx
            .data(|d| d.get_temp::<Option<(f64, f64)>>(egui::Id::new("pointer_jog")))
            .flatten();
        let direction = if !ctx.input(|i| i.focused) || self.chooser || self.settings_open {
            None
        } else if pointer.is_some() {
            pointer
        } else if !ctx.egui_wants_keyboard_input() {
            ctx.input(|i| {
                let x = i.key_down(egui::Key::ArrowRight) as i32
                    - i.key_down(egui::Key::ArrowLeft) as i32;
                let y =
                    i.key_down(egui::Key::ArrowUp) as i32 - i.key_down(egui::Key::ArrowDown) as i32;
                if x != 0 || y != 0 {
                    Some((x as f64, y as f64))
                } else {
                    None
                }
            })
        } else {
            None
        };
        if let Some((x, y)) =
            direction.filter(|_| self.snapshot.can_jog() && !self.snapshot.status.torch)
        {
            self.controller.send(Action::Jog {
                x,
                y,
                speed: self.settings.jog_speed,
            });
            self.was_jogging = true;
        } else if self.was_jogging {
            self.controller.send(Action::CancelJog);
            self.was_jogging = false;
        }
    }
    fn preview(&mut self, ui: &mut egui::Ui, height: f32) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("TOOLPATH").color(CYAN).strong());
            ui.small("Cut / torch on");
            ui.colored_label(ORANGE, "Rapid");
            if ui.button("Fit").clicked() {
                self.zoom = 1.0;
                self.pan = Vec2::ZERO;
            }
            ui.small("Wheel: zoom · drag: pan");
        });
        let (response, painter) =
            ui.allocate_painter(Vec2::new(ui.available_width(), height), egui::Sense::drag());
        let rect = response.rect;
        painter.rect_filled(rect, 6.0, Color32::from_rgb(12, 18, 25));
        if response.dragged() {
            self.pan += response.drag_delta();
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            self.zoom = (self.zoom * (scroll * 0.002).exp()).clamp(0.1, 100.0);
        }
        let Some(program) = &self.snapshot.program else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Open a G-code file to view its toolpath",
                egui::FontId::proportional(17.0),
                Color32::GRAY,
            );
            return;
        };
        let mut min = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mut max = min;
        for point in program.blocks.iter().flat_map(|b| &b.path) {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
        }
        let scale = ((rect.width() - 50.0) / (max.x - min.x).max(10.0) as f32)
            .min((rect.height() - 40.0) / (max.y - min.y).max(10.0) as f32)
            * self.zoom;
        let center = Point {
            x: (min.x + max.x) / 2.0,
            y: (min.y + max.y) / 2.0,
            z: 0.0,
        };
        let project = |p: Point| {
            Pos2::new(
                rect.center().x + (p.x - center.x) as f32 * scale + self.pan.x,
                rect.center().y - (p.y - center.y) as f32 * scale + self.pan.y,
            )
        };
        let painter = painter.with_clip_rect(rect);
        let spacing = 10f64.powf((60.0 / scale as f64).log10().floor()).max(0.001);
        if ((max.x - min.x) / spacing + (max.y - min.y) / spacing) < 300.0 {
            for i in (min.x / spacing).floor() as i64..=(max.x / spacing).ceil() as i64 {
                let x = i as f64 * spacing;
                painter.line_segment(
                    [
                        project(Point {
                            x,
                            y: min.y,
                            ..Default::default()
                        }),
                        project(Point {
                            x,
                            y: max.y,
                            ..Default::default()
                        }),
                    ],
                    Stroke::new(1.0_f32, Color32::from_gray(35)),
                );
            }
            for i in (min.y / spacing).floor() as i64..=(max.y / spacing).ceil() as i64 {
                let y = i as f64 * spacing;
                painter.line_segment(
                    [
                        project(Point {
                            x: min.x,
                            y,
                            ..Default::default()
                        }),
                        project(Point {
                            x: max.x,
                            y,
                            ..Default::default()
                        }),
                    ],
                    Stroke::new(1.0_f32, Color32::from_gray(35)),
                );
            }
        }
        let origin = project(Point::default());
        painter.line_segment(
            [origin - Vec2::new(8.0, 0.0), origin + Vec2::new(8.0, 0.0)],
            Stroke::new(1.0_f32, GREEN),
        );
        painter.line_segment(
            [origin - Vec2::new(0.0, 8.0), origin + Vec2::new(0.0, 8.0)],
            Stroke::new(1.0_f32, GREEN),
        );
        for (index, block) in program.blocks.iter().enumerate() {
            let active = self.snapshot.active == Some(index);
            let completed = self.snapshot.active.is_some_and(|n| index < n);
            let color = if active {
                Color32::WHITE
            } else if completed {
                Color32::from_rgb(65, 100, 110)
            } else if block.rapid {
                ORANGE
            } else if block.torch {
                CYAN
            } else {
                Color32::from_rgb(110, 135, 150)
            };
            for points in block.path.windows(2) {
                painter.line_segment(
                    [project(points[0]), project(points[1])],
                    Stroke::new(if active { 2.8_f32 } else { 1.5_f32 }, color),
                );
            }
        }
        if self.snapshot.status.position_known {
            let pos = project(self.snapshot.status.wpos);
            painter.circle_stroke(pos, 6.0, Stroke::new(2.0_f32, GREEN));
            painter.circle_filled(pos, 2.0, GREEN);
        }
        painter.text(
            rect.left_bottom() + Vec2::new(10.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{:.1} × {:.1} mm", max.x - min.x, max.y - min.y),
            egui::FontId::monospace(12.0),
            Color32::GRAY,
        );
    }
    fn code_view(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("G-CODE").color(CYAN).strong());
            if ui
                .selectable_value(&mut self.show_original, false, "Uploaded")
                .changed()
            {
                self.scroll_to_active = true;
            }
            if ui
                .selectable_value(&mut self.show_original, true, "Original")
                .changed()
            {
                self.scroll_to_active = true;
            }
            if ui.checkbox(&mut self.auto_scroll, "Auto-scroll").changed() {
                self.scroll_to_active = true;
            }
        });
        let Some(program) = &self.snapshot.program else {
            return;
        };
        for warning in &program.warnings {
            ui.colored_label(ORANGE, warning);
        }
        let active = self.snapshot.active.map(|i| {
            if self.show_original {
                program.blocks[i].source
            } else {
                i
            }
        });
        let changed = self.last_active != active;
        self.last_active = active;
        let count = if self.show_original {
            program.source_lines.len()
        } else {
            program.blocks.len()
        };
        let columns = if self.show_original {
            program.max_source_columns
        } else {
            program.max_uploaded_columns
        };
        let code_width = columns as f32 * 8.0 + 12.0;
        let height = 20.0;
        let stride = height + ui.spacing().item_spacing.y;
        let available = ui.available_height().max(60.0);
        let mut scroll = egui::ScrollArea::both()
            .id_salt("code")
            .auto_shrink([false, false]);
        if self.auto_scroll
            && (changed || self.scroll_to_active)
            && let Some(row) = active
        {
            scroll = scroll.vertical_scroll_offset(
                (row as f32 * stride + height / 2.0 - available / 2.0).max(0.0),
            );
        }
        self.scroll_to_active = false;
        scroll.show_rows(ui, height, count, |ui, rows| {
            for row in rows {
                let text = if self.show_original {
                    format!("{:>5}  {}", row + 1, program.source_lines[row])
                } else {
                    let b = &program.blocks[row];
                    format!(
                        "N{:<5} {}{}",
                        b.n,
                        b.code,
                        if b.inserted { "   ; pierce delay" } else { "" }
                    )
                };
                let (rect, _) = ui.allocate_exact_size(
                    Vec2::new(ui.available_width().max(code_width), height),
                    egui::Sense::hover(),
                );
                if active == Some(row) {
                    ui.painter()
                        .rect_filled(rect, 2.0, Color32::from_rgb(30, 85, 100));
                }
                ui.painter().text(
                    rect.left_center() + Vec2::new(4.0, 0.0),
                    egui::Align2::LEFT_CENTER,
                    text,
                    egui::FontId::monospace(13.0),
                    if active == Some(row) {
                        Color32::WHITE
                    } else {
                        Color32::from_gray(190)
                    },
                );
            }
        });
    }
    fn open_chooser(&mut self) {
        if !self.snapshot.job && !self.snapshot.transferring {
            self.chooser = true;
            self.chooser_path = self.settings.last_dir.to_string_lossy().into();
        }
    }
    fn dialogs(&mut self, root: &mut egui::Ui) {
        let ctx = root.ctx().clone();
        if self.chooser {
            let mut open = true;
            let mut selected = None;
            egui::Window::new("Open G-code — uploads automatically")
                .open(&mut open)
                .default_width(640.0)
                .show(&ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Path");
                        ui.add(
                            egui::TextEdit::singleline(&mut self.chooser_path).desired_width(450.0),
                        );
                        if ui.button("Open").clicked() {
                            selected = Some(PathBuf::from(&self.chooser_path));
                        }
                    });
                    let path = PathBuf::from(&self.chooser_path);
                    let directory = if path.is_dir() {
                        path.clone()
                    } else {
                        path.parent().unwrap_or(&self.settings.last_dir).to_owned()
                    };
                    if ui.button("↑ Parent folder").clicked()
                        && let Some(parent) = directory.parent()
                    {
                        self.chooser_path = parent.to_string_lossy().into();
                    }
                    egui::ScrollArea::vertical()
                        .max_height(370.0)
                        .show(ui, |ui| match std::fs::read_dir(&directory) {
                            Ok(entries) => {
                                let mut entries: Vec<_> = entries.filter_map(Result::ok).collect();
                                entries.sort_by_key(|e| (!e.path().is_dir(), e.file_name()));
                                for entry in entries {
                                    let path = entry.path();
                                    let dir = path.is_dir();
                                    let name = entry.file_name().to_string_lossy().into_owned();
                                    if name.starts_with('.') {
                                        continue;
                                    }
                                    if ui
                                        .selectable_label(
                                            false,
                                            format!(
                                                "{}{}",
                                                if dir { "[folder] " } else { "" },
                                                name
                                            ),
                                        )
                                        .clicked()
                                    {
                                        if dir {
                                            self.chooser_path = path.to_string_lossy().into();
                                        } else {
                                            selected = Some(path);
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                ui.colored_label(RED, e.to_string());
                            }
                        });
                });
            self.chooser = open;
            if let Some(path) = selected {
                if path.is_dir() {
                    self.chooser_path = path.to_string_lossy().into();
                } else {
                    self.open(path);
                }
            }
        }
        if self.settings_open {
            let mut open = true;
            let mut apply = false;
            egui::Window::new("Settings").open(&mut open).default_width(430.0).show(&ctx,|ui|{
                ui.add_enabled_ui(!self.demo,|ui|{
                    ui.label("Controller TCP address");ui.text_edit_singleline(&mut self.settings.address);
                    ui.label("Controller HTTP URL (file upload)");ui.text_edit_singleline(&mut self.settings.http);
                });
                ui.horizontal(|ui|{ui.label("Pierce delay");ui.add(egui::DragValue::new(&mut self.settings.pierce).range(0.0..=60.0).speed(0.05).suffix(" s"));});
                ui.small("Applied to every ignition and torch restoration on Resume.");
                if ui.add_enabled(!self.snapshot.job && !self.snapshot.transferring,egui::Button::new("Apply settings")).clicked(){apply=true;}
                ui.small("Changing pierce delay automatically regenerates and uploads the loaded job.");
            });
            self.settings_open = open;
            if apply {
                if self.settings.address.trim().is_empty()
                    || !self.settings.http.starts_with("http://")
                {
                    self.error = Some("Enter host:port and an http:// URL".into());
                } else {
                    self.controller.send(Action::Connect {
                        address: self.settings.address.clone(),
                        http: self.settings.http.clone(),
                    });
                    if (self.applied_pierce - self.settings.pierce).abs() > 1e-9
                        && let Some(p) = &self.snapshot.program
                    {
                        self.controller.send(Action::Load {
                            name: self.snapshot.name.clone(),
                            text: p.original.clone(),
                            pierce: self.settings.pierce,
                        });
                    }
                    self.applied_pierce = self.settings.pierce;
                    if !self.demo {
                        self.settings.save();
                    }
                    self.settings_open = false;
                }
            }
        }
        if self.log_open {
            egui::Window::new("Controller log")
                .open(&mut self.log_open)
                .default_size([700.0, 300.0])
                .show(&ctx, |ui| {
                    egui::ScrollArea::both()
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for line in &self.snapshot.log {
                                ui.monospace(line);
                            }
                        });
                });
        }
    }
}
fn duration(seconds: f64) -> String {
    if !seconds.is_finite() {
        return "—".into();
    }
    let s = seconds.max(0.0).round() as u64;
    format!("{:02}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
}
