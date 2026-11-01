use crate::{
    controller::{Action, Controller, Snapshot},
    gcode::{Point, Program},
};
use egui::{Color32, Pos2, RichText, Stroke, Vec2};
use serde::{Deserialize, Serialize};
use std::{
    path::PathBuf,
    sync::{
        Arc,
        mpsc::{Receiver, TryRecvError},
    },
};

const BACKGROUND: Color32 = Color32::from_rgb(8, 10, 13);
const SURFACE: Color32 = Color32::from_rgb(22, 26, 32);
const TEXT: Color32 = Color32::from_rgb(245, 247, 250);
const MUTED: Color32 = Color32::from_rgb(199, 205, 214);
const BORDER: Color32 = Color32::from_rgb(125, 136, 151);
const CYAN: Color32 = Color32::from_rgb(0, 229, 255);
const GREEN: Color32 = Color32::from_rgb(124, 255, 160);
const ORANGE: Color32 = Color32::from_rgb(255, 196, 102);
const RED: Color32 = Color32::from_rgb(255, 129, 129);
const CUT: Color32 = Color32::from_rgb(255, 230, 0);
const FEED_STROKE: Stroke = Stroke {
    width: 2.0,
    color: CUT,
};
const RAPID_STROKE: Stroke = Stroke {
    width: 1.2,
    color: CYAN,
};
const BLUE: Color32 = Color32::from_rgb(7, 78, 173);
const RUN_FILL: Color32 = Color32::from_rgb(8, 112, 49);
const STOP_FILL: Color32 = Color32::from_rgb(183, 29, 37);
const COMMENT: Color32 = Color32::from_rgb(150, 157, 167);
const LINE_NUMBER: Color32 = Color32::from_rgb(119, 130, 145);
const CODE_FONT_SIZE: f32 = 14.0;
const JOG_BUTTON_SIZE: f32 = 48.0;

fn gcode_color(letter: char) -> Color32 {
    match letter.to_ascii_uppercase() {
        'G' => TEXT,
        'M' => Color32::from_rgb(255, 151, 145),
        'X' | 'Y' => CYAN,
        'Z' => Color32::from_rgb(145, 232, 170),
        'F' => Color32::from_rgb(255, 224, 112),
        'I' => Color32::from_rgb(198, 224, 136),
        'J' => Color32::from_rgb(108, 209, 159),
        'K' => Color32::from_rgb(168, 204, 255),
        'R' => Color32::from_rgb(255, 202, 155),
        'S' => Color32::from_rgb(196, 170, 255),
        'T' => Color32::from_rgb(206, 180, 245),
        'P' => Color32::from_rgb(239, 184, 225),
        'N' => Color32::from_rgb(156, 178, 201),
        'A' => Color32::from_rgb(244, 170, 115),
        'B' => Color32::from_rgb(170, 225, 135),
        'C' => Color32::from_rgb(195, 173, 235),
        'D' => Color32::from_rgb(125, 200, 180),
        'E' => Color32::from_rgb(220, 202, 160),
        'H' => Color32::from_rgb(181, 200, 240),
        'L' => Color32::from_rgb(226, 169, 203),
        'O' => Color32::from_rgb(190, 206, 154),
        'Q' => Color32::from_rgb(216, 184, 150),
        'U' => Color32::from_rgb(246, 184, 171),
        'V' => Color32::from_rgb(166, 219, 205),
        'W' => Color32::from_rgb(205, 186, 223),
        _ => TEXT,
    }
}

fn highlight_gcode(source: &str) -> egui::text::LayoutJob {
    let mut job = egui::text::LayoutJob::default();
    let mut chars = source.char_indices().peekable();
    while let Some((start, ch)) = chars.next() {
        let color = match ch {
            ';' => {
                chars.by_ref().for_each(|_| {});
                COMMENT
            }
            '(' => {
                let mut depth = 1;
                for (_, ch) in chars.by_ref() {
                    match ch {
                        '(' => depth += 1,
                        ')' => depth -= 1,
                        _ => {}
                    }
                    if depth == 0 {
                        break;
                    }
                }
                COMMENT
            }
            ch if ch.is_ascii_alphabetic() => {
                while chars.peek().is_some_and(|(_, ch)| {
                    ch.is_ascii_whitespace() || ch.is_ascii_digit() || matches!(ch, '+' | '-' | '.')
                }) {
                    chars.next();
                }
                gcode_color(ch)
            }
            _ => TEXT,
        };
        let end = chars.peek().map_or(source.len(), |(index, _)| *index);
        job.append(
            &source[start..end],
            0.0,
            egui::TextFormat {
                font_id: egui::FontId::monospace(CODE_FONT_SIZE),
                color,
                ..Default::default()
            },
        );
    }
    job
}

fn console_output(
    ui: &mut egui::Ui,
    snapshot: &Snapshot,
    show_status: bool,
) -> egui::scroll_area::ScrollAreaOutput<()> {
    let lines = if show_status {
        &snapshot.console
    } else {
        &snapshot.console_output
    };
    let font = egui::TextStyle::Monospace.resolve(ui.style());
    let row_height = ui.fonts_mut(|fonts| fonts.row_height(&font));
    let columns = lines
        .iter()
        .map(|line| line.chars().count())
        .max()
        .unwrap_or(0);
    let width = columns as f32 * ui.fonts_mut(|fonts| fonts.glyph_width(&font, 'M'));
    egui::ScrollArea::both()
        .id_salt(("console_output", show_status))
        .auto_shrink([false, false])
        .max_height((ui.available_height() - 65.0).max(80.0))
        .stick_to_bottom(true)
        .show_rows(ui, row_height, lines.len(), |ui, rows| {
            // Keep the horizontal extent stable when the longest line is
            // outside the visible rows. Lines scroll horizontally, never wrap.
            ui.set_min_width(width);
            for row in rows {
                ui.add(
                    egui::Label::new(RichText::new(&lines[row]).monospace())
                        .wrap_mode(egui::TextWrapMode::Extend)
                        .selectable(true),
                );
            }
        })
}

fn configure_style(ctx: &egui::Context) {
    ctx.global_style_mut(|style| {
        style.visuals = high_contrast_visuals();
        style.spacing.interact_size = Vec2::new(40.0, 30.0);
        style.spacing.button_padding = Vec2::new(9.0, 5.0);
        style.spacing.icon_width = 18.0;
        style.spacing.scroll = egui::style::ScrollStyle::solid();
        style.spacing.scroll.bar_width = 8.0;
        style.spacing.scroll.handle_min_length = 28.0;
        style.spacing.scroll.foreground_color = true;
        for (text_style, size) in [
            (egui::TextStyle::Body, 15.0),
            (egui::TextStyle::Button, 15.0),
            (egui::TextStyle::Small, 11.0),
            (egui::TextStyle::Heading, 20.0),
        ] {
            style
                .text_styles
                .insert(text_style, egui::FontId::proportional(size));
        }
        style.text_styles.insert(
            egui::TextStyle::Monospace,
            egui::FontId::monospace(CODE_FONT_SIZE),
        );
    });
}

fn high_contrast_visuals() -> egui::Visuals {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = BACKGROUND;
    visuals.window_fill = SURFACE;
    visuals.extreme_bg_color = BACKGROUND;
    visuals.faint_bg_color = SURFACE;
    visuals.code_bg_color = BACKGROUND;
    visuals.weak_text_color = Some(MUTED);
    visuals.window_stroke = Stroke::new(1.0_f32, BORDER);
    visuals.selection.bg_fill = BLUE;
    visuals.selection.stroke = Stroke::new(1.0_f32, TEXT);
    visuals.hyperlink_color = CYAN;
    visuals.warn_fg_color = ORANGE;
    visuals.error_fg_color = RED;
    visuals.disabled_alpha = 0.55;
    for widget in [
        &mut visuals.widgets.noninteractive,
        &mut visuals.widgets.inactive,
        &mut visuals.widgets.hovered,
        &mut visuals.widgets.active,
        &mut visuals.widgets.open,
    ] {
        widget.bg_fill = SURFACE;
        widget.weak_bg_fill = SURFACE;
        widget.bg_stroke = Stroke::new(1.0_f32, BORDER);
        widget.fg_stroke = Stroke::new(1.0_f32, TEXT);
        widget.corner_radius = egui::CornerRadius::same(3);
    }
    visuals.widgets.hovered.bg_fill = Color32::from_rgb(47, 55, 67);
    visuals.widgets.hovered.weak_bg_fill = visuals.widgets.hovered.bg_fill;
    visuals.widgets.hovered.bg_stroke.color = TEXT;
    visuals.widgets.active.bg_fill = BLUE;
    visuals.widgets.active.weak_bg_fill = BLUE;
    visuals.widgets.active.bg_stroke.color = CYAN;
    visuals.widgets.open = visuals.widgets.active;
    visuals
}

// Draw a shared edge across the whole panel, including its content margins.
// Keep the vertical clip so scrolled dividers cannot overlap the fixed controls.
fn panel_divider(ui: &mut egui::Ui, columns: egui::emath::Rangef, gap: f32) {
    let clip = egui::Rect::from_x_y_ranges(columns, ui.clip_rect().y_range());
    ui.painter().with_clip_rect(clip).hline(
        columns,
        ui.cursor().top() + gap / 2.0,
        Stroke::new(1.0_f32, BORDER),
    );
    ui.add_space(gap);
}

fn toolpath_legend(painter: &egui::Painter, plot: egui::Rect) {
    let font = egui::FontId::proportional(14.0);
    let key = painter.layout_no_wrap("Key".into(), font.clone(), MUTED);
    let entries = [("Feed", FEED_STROKE), ("Rapid", RAPID_STROKE)].map(|(label, stroke)| {
        (
            painter.layout_no_wrap(label.into(), font.clone(), TEXT),
            stroke,
        )
    });
    let width = key.size().x
        + 14.0
        + entries
            .iter()
            .map(|(label, _)| 30.0 + 7.0 + label.size().x)
            .sum::<f32>()
        + 16.0;
    let height = key.size().y.max(
        entries
            .iter()
            .map(|(label, _)| label.size().y)
            .fold(0.0, f32::max),
    );
    let rect = egui::Rect::from_min_size(
        plot.right_bottom() - Vec2::new(width + 10.0, height + 8.0),
        Vec2::new(width, height),
    );
    // A plain, non-interactive key; the backing keeps paths from crossing its text.
    painter.rect_filled(rect.expand(4.0), 0.0, Color32::BLACK);
    painter.galley(rect.left_top(), key.clone(), MUTED);
    let mut x = rect.left() + key.size().x + 14.0;
    for (label, stroke) in entries {
        painter.line_segment(
            [
                Pos2::new(x, rect.center().y),
                Pos2::new(x + 30.0, rect.center().y),
            ],
            stroke,
        );
        painter.galley(Pos2::new(x + 37.0, rect.top()), label.clone(), TEXT);
        x += 37.0 + label.size().x + 16.0;
    }
}

fn shortcut_hint(shortcut: &str) -> RichText {
    RichText::new(shortcut)
        .monospace()
        .size(12.0)
        .color(TEXT)
        .background_color(BACKGROUND)
        .underline()
}

fn shortcut_button(label: &str, shortcut: &str) -> egui::Button<'static> {
    egui::Button::new((RichText::new(label), shortcut_hint(shortcut))).gap(7.0)
}

#[derive(Clone, Copy)]
enum TransportIcon {
    Play,
    Pause,
    Stop,
}

fn transport_button(
    ui: &mut egui::Ui,
    enabled: bool,
    label: &str,
    shortcut: &str,
    icon: TransportIcon,
    fill: Option<Color32>,
) -> egui::Response {
    let icon_id = ui.next_auto_id().with("transport_icon");
    ui.add_enabled_ui(enabled, |ui| {
        let mut button = egui::Button::new((
            egui::Atom::custom(icon_id, Vec2::splat(14.0)),
            RichText::new(label),
            shortcut_hint(shortcut),
        ))
        .gap(4.0)
        .min_size(Vec2::new(100.0, 44.0));
        if let Some(fill) = fill {
            button = button.fill(fill);
        }
        let output = button.atom_ui(ui);
        if let Some(rect) = output.rect(icon_id) {
            let rect = rect.shrink(1.0);
            match icon {
                TransportIcon::Play => {
                    ui.painter().add(egui::Shape::convex_polygon(
                        vec![rect.left_top(), rect.right_center(), rect.left_bottom()],
                        TEXT,
                        Stroke::NONE,
                    ));
                }
                TransportIcon::Pause => {
                    for x in [rect.left(), rect.right() - 4.0] {
                        ui.painter().rect_filled(
                            egui::Rect::from_min_size(
                                Pos2::new(x, rect.top()),
                                Vec2::new(4.0, rect.height()),
                            ),
                            0.0,
                            TEXT,
                        );
                    }
                }
                TransportIcon::Stop => {
                    ui.painter().rect_filled(rect, 0.0, TEXT);
                }
            }
        }
        output.response
    })
    .inner
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub address: String,
    pub http: String,
    pub jog_speed: f64,
    pub jog_increment: f64,
    pub text_size: f32,
    pub last_dir: PathBuf,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            address: "192.168.0.1:23".into(),
            http: "http://192.168.0.1".into(),
            jog_speed: 600.0,
            jog_increment: 1.0,
            text_size: 13.0,
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
    focus_control: Option<char>,
    show_original: bool,
    auto_scroll: bool,
    last_active: Option<usize>,
    scroll_to_active: bool,
    picker: Option<Receiver<anyhow::Result<Option<PathBuf>>>>,
    picker_purpose: crate::file_dialog::Purpose,
    files_open: bool,
    remote_filename: String,
    native_window: u32,
    settings_open: bool,
    log_open: bool,
    console_open: bool,
    console_focus: bool,
    console_input_id: Option<egui::Id>,
    console_submit: bool,
    console_input: String,
    console_history: Vec<String>,
    console_index: usize,
    console_draft: String,
    console_status: bool,
    show_hull: bool,
    hull_points: Vec<Point>,
    hull_program: Option<Arc<Program>>,
    error: Option<String>,
    was_jogging: bool,
    jog_interrupted: bool,
    zoom: f32,
    pan: Vec2,
    demo: bool,
}
impl App {
    pub fn new(mut settings: Settings, demo: bool) -> Self {
        settings.text_size = if settings.text_size.is_finite() {
            settings.text_size.clamp(8.0, 40.0)
        } else {
            13.0
        };
        let mut controller = Controller::start(
            settings.address.clone(),
            settings.http.clone(),
            (!demo).then(crate::coordinates::Coordinates::path),
        );
        if !demo {
            controller.enable_wifi_rescans();
        }
        Self {
            controller,
            snapshot: Snapshot::default(),
            settings,
            dro: ["0.000".into(), "0.000".into()],
            dro_edit: [false; 2],
            focus_control: None,
            show_original: true,
            auto_scroll: true,
            last_active: None,
            scroll_to_active: false,
            picker: None,
            picker_purpose: crate::file_dialog::Purpose::Gcode,
            files_open: false,
            remote_filename: "config.yaml".into(),
            native_window: 0,
            settings_open: false,
            log_open: false,
            console_open: false,
            console_focus: false,
            console_input_id: None,
            console_submit: false,
            console_input: String::new(),
            console_history: Vec::new(),
            console_index: 0,
            console_draft: String::new(),
            console_status: false,
            show_hull: false,
            hull_points: Vec::new(),
            hull_program: None,
            error: None,
            was_jogging: false,
            jog_interrupted: false,
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
                });
                self.zoom = 1.0;
                self.pan = Vec2::ZERO;
                self.error = None;
            }
            Err(e) => self.error = Some(format!("{}: {e}", path.display())),
        }
    }
    pub fn load_text(&self, name: &str, text: &str) {
        self.controller.send(Action::Load {
            name: name.into(),
            text: text.into(),
        });
    }
    pub fn ui(&mut self, root: &mut egui::Ui) {
        self.snapshot = self.controller.snapshot();
        if let Some(program) = &self.snapshot.program
            && !self
                .hull_program
                .as_ref()
                .is_some_and(|old| Arc::ptr_eq(old, program))
        {
            self.hull_points = program.hull();
            self.hull_program = Some(program.clone());
        }
        self.native_window = root
            .ctx()
            .data(|d| d.get_temp(egui::Id::new("native_window")))
            .unwrap_or(0);
        if let Some(picker) = &self.picker {
            match picker.try_recv() {
                Ok(result) => {
                    self.picker = None;
                    match result {
                        Ok(Some(path)) => match self.picker_purpose.clone() {
                            crate::file_dialog::Purpose::Gcode => self.open(path),
                            purpose => {
                                if let Some(parent) = path.parent() {
                                    self.settings.last_dir = parent.to_owned();
                                    if !self.demo {
                                        self.settings.save();
                                    }
                                }
                                let action = match purpose {
                                    crate::file_dialog::Purpose::Upload => {
                                        if let Some(name) =
                                            path.file_name().and_then(|n| n.to_str())
                                        {
                                            self.remote_filename = name.into();
                                        }
                                        Action::UploadFile { path }
                                    }
                                    crate::file_dialog::Purpose::Download(name) => {
                                        Action::DownloadFile { name, path }
                                    }
                                    crate::file_dialog::Purpose::Gcode => unreachable!(),
                                };
                                self.controller.send(action);
                            }
                        },
                        Ok(None) => {}
                        Err(error) => self.error = Some(format!("{error:#}")),
                    }
                }
                Err(TryRecvError::Disconnected) => {
                    self.picker = None;
                    self.error = Some("System file chooser exited unexpectedly".into());
                }
                Err(TryRecvError::Empty) => {}
            }
        }
        let ctx = root.ctx().clone();
        self.update_ui_zoom(&ctx);
        configure_style(&ctx);
        if self.console_open
            && self
                .console_input_id
                .is_some_and(|id| ctx.memory(|memory| memory.has_focus(id)))
        {
            // Keep history arrows inside the console, but let TextEdit process
            // preceding text before applying history below. Consume Enter here
            // so it cannot activate a machine button earlier in the UI pass.
            if ctx.input(|input| {
                input.key_pressed(egui::Key::ArrowUp) || input.key_pressed(egui::Key::ArrowDown)
            }) {
                ctx.memory_mut(|memory| memory.move_focus(egui::FocusDirection::None));
            }
            self.console_submit =
                ctx.input_mut(|input| input.consume_key(egui::Modifiers::NONE, egui::Key::Enter));
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::O)) {
            self.open_chooser();
        }
        if !ctx.egui_wants_keyboard_input()
            && !ctx.input(|input| input.key_pressed(egui::Key::Escape))
            && ctx.input(|i| i.focused)
            && self.picker.is_none()
            && !self.settings_open
            && !self.log_open
            && !self.files_open
            && let Some(shortcut) = ctx.input_mut(|i| take_shortcut(&mut i.events))
        {
            match shortcut {
                'G' | 'M' => {
                    self.console_input = shortcut.to_string();
                    self.console_draft = self.console_input.clone();
                    self.console_index = self.console_history.len();
                    self.console_open = true;
                    self.console_focus = true;
                }
                'X' | 'Y' if self.can_set_coordinates() => {
                    self.console_open = false;
                    self.focus_control = Some(shortcut);
                }
                'F' | 'I' => {
                    self.console_open = false;
                    self.focus_control = Some(shortcut);
                }
                'O' => self.open_chooser(),
                'R' => {
                    if self.resume_ready() {
                        self.controller.send(Action::Resume);
                    } else if self.snapshot.can_run() {
                        self.controller.send(Action::Run);
                    }
                }
                'P' => {
                    if self.resume_ready() {
                        self.controller.send(Action::Resume);
                    } else if self.snapshot.job
                        && self.snapshot.connected
                        && !self.snapshot.status.state.starts_with("Hold")
                    {
                        self.controller.send(Action::Pause);
                    }
                }
                'S' if self.snapshot.connected => self.controller.send(Action::Stop),
                _ => {}
            }
        }
        egui::Panel::top("header").show_inside(root, |ui| {
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                ui.heading(RichText::new("plasma-wifi").strong());
                ui.separator();
                let (text, color) = self.state_label();
                let badge = ui.painter().add(egui::Shape::Noop);
                let state = ui.label(RichText::new(text).size(24.0).strong().color(BACKGROUND));
                ui.painter().set(
                    badge,
                    egui::Shape::rect_filled(state.rect.expand2(Vec2::new(3.0, 1.0)), 3.0, color),
                );
                if self.demo {
                    ui.label(RichText::new("SIMULATED CONTROLLER").color(ORANGE));
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Settings").clicked() {
                        self.settings_open = true;
                    }
                    if ui
                        .add(shortcut_button("Console", "G/M"))
                        .on_hover_text("G / M: open the console with that prefix · Esc: close")
                        .clicked()
                    {
                        self.console_open = !self.console_open;
                        self.console_focus = self.console_open;
                    }
                    if ui.button("Log").clicked() {
                        self.log_open = !self.log_open;
                    }
                    if ui.button("Files").clicked() {
                        self.files_open = !self.files_open;
                        if self.files_open && self.snapshot.can_manage_files() {
                            self.controller.send(Action::ListFiles);
                        }
                    }
                    if ui
                        .add_enabled(
                            !self.snapshot.job
                                && !self.snapshot.tracing
                                && !self.snapshot.transferring
                                && self.picker.is_none(),
                            shortcut_button("Open G-code…", "O / Ctrl+O"),
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
            let error = self
                .error
                .as_ref()
                .or(self.snapshot.alarm.as_ref())
                .or(self.snapshot.error.as_ref());
            if let Some(error) = error {
                ui.colored_label(RED, error);
            }
        });
        egui::Panel::left("controls")
            .resizable(false)
            .exact_size(344.0)
            .show_inside(root, |ui| {
                self.controls(ui);
            });
        egui::CentralPanel::default().show_inside(root, |ui| {
            let preview_height = (ui.available_height() * 0.52).max(140.0);
            self.preview(ui, preview_height);
            panel_divider(ui, ui.max_rect().x_range().expand(8.0), 5.0);
            self.code_view(ui);
        });
        root.painter().rect_stroke(
            root.max_rect(),
            0.0,
            Stroke::new(1.0_f32, BORDER),
            egui::StrokeKind::Inside,
        );
        self.dialogs(root);
        self.jog_input(&ctx);
    }
    fn state_label(&self) -> (String, Color32) {
        let state = &self.snapshot.status.state;
        if !self.snapshot.connected {
            return ("DISCONNECTED".into(), RED);
        }
        if self.snapshot.tracing {
            return ("TRACING HULL".into(), CYAN);
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
    fn can_set_coordinates(&self) -> bool {
        self.snapshot.connected
            && self.snapshot.initialized
            && self.snapshot.status.idle()
            && !self.snapshot.job
            && self.snapshot.status.position_known
            && !self.snapshot.transferring
            && !self.snapshot.tracing
            && !self.snapshot.jog_restarting
    }
    fn resume_ready(&self) -> bool {
        self.snapshot.job
            && self.snapshot.connected
            && self.snapshot.status.state.starts_with("Hold")
            && self.snapshot.phase == "Paused · torch off"
    }
    fn controls(&mut self, ui: &mut egui::Ui) {
        let columns = ui.max_rect().x_range().expand(8.0);
        ui.add_space(8.0);
        ui.label(RichText::new("WORK POSITION  /  mm").strong());
        let can_set = self.can_set_coordinates();
        let position = self.snapshot.status.wpos;
        for (i, axis, value) in [(0, 'X', position.x), (1, 'Y', position.y)] {
            if !self.dro_edit[i] {
                self.dro[i] = format!("{value:.3}");
            }
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(
                        can_set,
                        egui::Button::new(format!("{axis}0")).min_size(Vec2::new(48.0, 40.0)),
                    )
                    .on_hover_text("Set this work coordinate to zero without moving")
                    .clicked()
                {
                    self.controller
                        .send(Action::SetCoordinate { axis, value: 0.0 });
                }
                let id = egui::Id::new(("work_coordinate", axis));
                if self.focus_control == Some(axis) && can_set {
                    self.focus_control = None;
                    ui.memory_mut(|memory| memory.request_focus(id));
                    let mut state = egui::TextEdit::load_state(ui.ctx(), id).unwrap_or_default();
                    state
                        .cursor
                        .set_char_range(Some(egui::text::CCursorRange::two(
                            egui::text::CCursor::new(0),
                            egui::text::CCursor::new(self.dro[i].chars().count()),
                        )));
                    state.store(ui.ctx(), id);
                }
                let response = ui
                    .add_enabled(
                        can_set,
                        egui::TextEdit::singleline(&mut self.dro[i])
                            .id(id)
                            .font(egui::FontId::monospace(26.0))
                            .desired_width(212.0),
                    )
                    .on_hover_text(format!(
                        "{axis}: focus this coordinate · Enter: apply a value or /2"
                    ));
                self.dro_edit[i] = response.has_focus();
                if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    let edited = if self.dro[i].trim() == "/2" {
                        Some(value / 2.0)
                    } else {
                        self.dro[i]
                            .trim()
                            .parse::<f64>()
                            .ok()
                            .filter(|value| value.is_finite())
                    };
                    match edited {
                        Some(value) => {
                            self.controller.send(Action::SetCoordinate { axis, value });
                            self.error = None;
                        }
                        _ => {
                            self.error =
                                Some("Enter a finite coordinate in millimetres, or /2".into())
                        }
                    }
                }
                if ui
                    .add_enabled(
                        can_set,
                        egui::Button::new("/2").min_size(Vec2::new(44.0, 40.0)),
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
        ui.horizontal(|ui| {
            ui.label("Velocity");
            let velocity = if self.snapshot.connected && self.snapshot.status.fresh() {
                format!("{:.0}", self.snapshot.status.feed)
            } else {
                "—".into()
            };
            ui.label(RichText::new(velocity).monospace().size(22.0).color(CYAN));
            ui.label("mm/min");
        })
        .response
        .on_hover_text(
            "Live tool speed reported by FluidNC, including acceleration and feed overrides.",
        );
        ui.small("Reported modes").on_hover_text("FluidNC's latest parser modes ($G / automatic reports). Parser lookahead can be ahead of physical motion; motion-only changes may be omitted while moving.");
        let modes = if self.snapshot.connected && !self.snapshot.modal.is_empty() {
            self.snapshot
                .modal
                .split_whitespace()
                .filter(|word| word.starts_with(['G', 'M', 'T']))
                .collect::<Vec<_>>()
                .join(" ")
        } else {
            "—".into()
        };
        ui.add(egui::Label::new(RichText::new(modes).monospace()).wrap());
        panel_divider(ui, columns, 12.0);
        ui.horizontal(|ui| {
            let resume_ready = self.resume_ready();
            if transport_button(
                ui,
                self.snapshot.can_run() || resume_ready,
                if self.snapshot.status.state.starts_with("Hold") {
                    "Resume"
                } else {
                    "RUN"
                },
                "R",
                TransportIcon::Play,
                Some(RUN_FILL),
            )
            .clicked()
            {
                self.controller.send(if resume_ready {
                    Action::Resume
                } else {
                    Action::Run
                });
            }
            let paused = self.snapshot.status.state.starts_with("Hold");
            if transport_button(
                ui,
                self.snapshot.job && self.snapshot.connected && (!paused || resume_ready),
                if paused { "Resume" } else { "Pause" },
                "P",
                if paused {
                    TransportIcon::Play
                } else {
                    TransportIcon::Pause
                },
                None,
            )
            .clicked()
            {
                self.controller.send(if paused {
                    Action::Resume
                } else {
                    Action::Pause
                });
            }
            if transport_button(
                ui,
                self.snapshot.connected,
                "STOP",
                "S",
                TransportIcon::Stop,
                Some(STOP_FILL),
            )
            .clicked()
            {
                self.controller.send(Action::Stop);
            }
        });
        if self.snapshot.status.state.contains("Alarm") && ui.button("Unlock alarm").clicked() {
            self.controller.send(Action::Unlock);
        }
        // Reserve this row so controls stay put as the job phase changes.
        ui.label(RichText::new(&self.snapshot.phase).color(ORANGE));
        let progress =
            ui.add(egui::ProgressBar::new(self.snapshot.progress as f32).show_percentage());
        ui.painter().rect_stroke(
            progress.rect,
            progress.rect.height() / 2.0,
            Stroke::new(1.0_f32, BORDER),
            egui::StrokeKind::Inside,
        );
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
        panel_divider(ui, columns, 12.0);
        egui::ScrollArea::vertical()
            .id_salt("motion_controls")
            .show(ui, |ui| self.motion_controls(ui, columns));
    }
    fn motion_controls(&mut self, ui: &mut egui::Ui, columns: egui::emath::Rangef) {
        ui.label(RichText::new("FEED OVERRIDE").strong());
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
                let button = egui::Button::new(label).min_size(Vec2::new(56.0, 32.0));
                let button = if byte == 0x90 {
                    button.fill(BLUE)
                } else {
                    button
                };
                if ui
                    .add_enabled(self.snapshot.connected && !self.snapshot.tracing, button)
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
        panel_divider(ui, columns, 12.0);
        ui.label(RichText::new("JOG  /  tap or hold").strong());
        if self.snapshot.connected
            && (self.snapshot.acceleration.iter().any(Option::is_none)
                || self.snapshot.planner_blocks.is_none())
        {
            ui.small("Waiting for controller acceleration / planner settings…");
        }
        ui.horizontal(|ui| {
            ui.label("Increment");
            ui.label(shortcut_hint("I"));
            if self.focus_control == Some('I') {
                self.focus_control = None;
                let id = ui.next_auto_id();
                ui.memory_mut(|memory| memory.request_focus(id));
            }
            if ui
                .add(
                    egui::DragValue::new(&mut self.settings.jog_increment)
                        .update_while_editing(false)
                        .custom_parser(|text| {
                            text.trim()
                                .parse::<f64>()
                                .ok()
                                .filter(|value| value.is_finite())
                        })
                        .range(0.001..=100.0)
                        .speed(0.1)
                        .suffix(" mm"),
                )
                .changed()
                && !self.demo
            {
                self.settings.save();
            }
        });
        ui.horizontal(|ui| {
            ui.label("Speed");
            ui.label(shortcut_hint("F"));
            if self.focus_control == Some('F') {
                self.focus_control = None;
                let id = ui.next_auto_id();
                ui.memory_mut(|memory| memory.request_focus(id));
            }
            if ui
                .add(
                    egui::DragValue::new(&mut self.settings.jog_speed)
                        .update_while_editing(false)
                        .custom_parser(|text| {
                            text.trim()
                                .parse::<f64>()
                                .ok()
                                .filter(|value| value.is_finite())
                        })
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
                ui.horizontal(|ui| {
                    let pad_width = 3.0 * JOG_BUTTON_SIZE + 2.0 * 5.0;
                    ui.add_space(((ui.available_width() - pad_width) / 2.0).max(0.0));
                    egui::Grid::new("jog_pad")
                        .spacing([5.0, 5.0])
                        .show(ui, |ui| {
                            for row in [
                                [("", 0.0, 0.0), ("Y+", 0.0, 1.0), ("", 0.0, 0.0)],
                                [("X−", -1.0, 0.0), ("", 0.0, 0.0), ("X+", 1.0, 0.0)],
                                [("", 0.0, 0.0), ("Y−", 0.0, -1.0), ("", 0.0, 0.0)],
                            ] {
                                for (label, x, y) in row {
                                    if label.is_empty() {
                                        ui.allocate_space(Vec2::splat(JOG_BUTTON_SIZE));
                                        continue;
                                    }
                                    let response = ui.add_sized(
                                        Vec2::splat(JOG_BUTTON_SIZE),
                                        egui::Button::new(RichText::new(label).strong()).fill(BLUE),
                                    );
                                    if response.is_pointer_button_down_on()
                                        && response.contains_pointer()
                                        || response.clicked()
                                    {
                                        let boosted = ui.input(|input| {
                                            input.modifiers.shift
                                                || input.events.iter().any(|event| {
                                                    matches!(
                                                        event,
                                                        egui::Event::PointerButton {
                                                            pressed: true,
                                                            modifiers,
                                                            ..
                                                        } if modifiers.shift
                                                    )
                                                })
                                        });
                                        pointer_jog = Some((x, y, boosted));
                                    }
                                }
                                ui.end_row();
                            }
                        });
                });
            },
        );
        ui.ctx()
            .data_mut(|d| d.insert_temp(egui::Id::new("pointer_jog"), pointer_jog));
        ui.small("Hold Shift: ×4 jog speed (max 3000 mm/min)");
        ui.small("Tap: one increment · release hold: stop");
        ui.small("Arrow keys: X/Y · Esc: cancel jog");
        ui.small(format!(
            "Torch: {}",
            if self.snapshot.status.torch {
                "ON"
            } else {
                "off"
            }
        ));
        panel_divider(ui, columns, 10.0);
        if self.snapshot.tracing {
            if ui.button("Cancel hull trace").clicked() {
                self.controller.send(Action::CancelTrace);
            }
        } else if ui
            .add_enabled(
                self.snapshot.can_jog()
                    && self.snapshot.status.idle()
                    && !self.snapshot.jog_restarting
                    && self.snapshot.program.is_some()
                    && !self.snapshot.status.torch,
                egui::Button::new("Trace hull at jog speed"),
            )
            .on_hover_text("Follow the job's convex hull with the torch off, then return here")
            .clicked()
        {
            self.show_hull = true;
            self.controller.send(Action::TraceHull {
                speed: self.settings.jog_speed,
                boosted: ui.input(|input| input.modifiers.shift),
            });
        }
        ui.small("X/Y: edit position · G/M: console");
    }
    fn jog_feed(&self, boosted: bool) -> f64 {
        (self.settings.jog_speed * if boosted { 4.0 } else { 1.0 }).min(3000.0)
    }
    fn jog_input(&mut self, ctx: &egui::Context) {
        if self.snapshot.tracing {
            self.controller.send(Action::TraceSpeed {
                boosted: ctx.input(|input| input.focused && input.modifiers.shift),
            });
        }
        if self.snapshot.status.state == "Jog" || self.snapshot.jog_restarting {
            self.controller.send(Action::JogSpeed {
                speed: self.jog_feed(ctx.input(|input| input.focused && input.modifiers.shift)),
            });
        }
        let pointer = ctx
            .data(|d| d.get_temp::<Option<(f64, f64, bool)>>(egui::Id::new("pointer_jog")))
            .flatten();
        let interrupted = !ctx.input(|i| i.focused)
            || self.picker.is_some()
            || self.settings_open
            || self.console_open
            || self.log_open
            || self.files_open
            || ctx.input(|i| i.key_down(egui::Key::Escape) || i.key_pressed(egui::Key::Escape))
            || !self.snapshot.can_jog()
            || self.snapshot.status.torch;
        let (directions, presses, boosted) = if interrupted {
            ([0, 0], [0, 0], false)
        } else if let Some((x, y, boosted)) = pointer {
            let raw = [x as i8, y as i8];
            let down = ctx.input(|i| i.pointer.primary_down());
            let pressed = ctx.input(|i| i.pointer.primary_pressed()) || !self.was_jogging && !down;
            (
                if down { raw } else { [0, 0] },
                if pressed { raw.map(i32::from) } else { [0, 0] },
                boosted,
            )
        } else if !ctx.egui_wants_keyboard_input() {
            ctx.input(|i| {
                let allowed = !i.modifiers.ctrl && !i.modifiers.alt && !i.modifiers.command;
                let directions = if allowed {
                    [
                        i8::from(i.key_down(egui::Key::ArrowRight))
                            - i8::from(i.key_down(egui::Key::ArrowLeft)),
                        i8::from(i.key_down(egui::Key::ArrowUp))
                            - i8::from(i.key_down(egui::Key::ArrowDown)),
                    ]
                } else {
                    [0, 0]
                };
                let mut presses = [0, 0];
                let mut boosted = i.modifiers.shift;
                for event in &i.events {
                    if let egui::Event::Key {
                        key,
                        pressed: true,
                        repeat: false,
                        modifiers,
                        ..
                    } = event
                        && !modifiers.ctrl
                        && !modifiers.alt
                        && !modifiers.command
                    {
                        if matches!(
                            key,
                            egui::Key::ArrowRight
                                | egui::Key::ArrowLeft
                                | egui::Key::ArrowUp
                                | egui::Key::ArrowDown
                        ) {
                            boosted = modifiers.shift;
                        }
                        match key {
                            egui::Key::ArrowRight => presses[0] += 1,
                            egui::Key::ArrowLeft => presses[0] -= 1,
                            egui::Key::ArrowUp => presses[1] += 1,
                            egui::Key::ArrowDown => presses[1] -= 1,
                            _ => {}
                        }
                    }
                }
                (directions, presses, boosted)
            })
        } else {
            ([0, 0], [0, 0], false)
        };
        if interrupted || ctx.egui_wants_keyboard_input() && pointer.is_none() {
            if self.was_jogging {
                self.controller.send(Action::CancelJog);
            }
            self.was_jogging = false;
        } else if (directions != [0, 0] || presses != [0, 0] || self.was_jogging)
            && self.snapshot.can_jog()
            && !self.snapshot.status.torch
        {
            self.controller.send(Action::JogInput {
                directions,
                presses,
                speed: self.jog_feed(boosted),
                step: self.settings.jog_increment,
            });
            self.was_jogging = directions != [0, 0];
        }
        // A released increment may still be completing when focus is lost.
        if interrupted && !self.jog_interrupted && !self.snapshot.tracing {
            self.controller.send(Action::CancelJog);
        }
        self.jog_interrupted = interrupted;
    }
    fn preview(&mut self, ui: &mut egui::Ui, height: f32) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("TOOLPATH").strong());
            if ui.button("Fit").clicked() {
                self.zoom = 1.0;
                self.pan = Vec2::ZERO;
            }
            ui.checkbox(&mut self.show_hull, "Hull");
            ui.small("Wheel: zoom · drag: pan");
        });
        let (response, painter) = ui.allocate_painter(
            Vec2::new(ui.available_width(), height),
            egui::Sense::click_and_drag(),
        );
        let rect = response.rect;
        painter.rect_filled(rect, 0.0, Color32::BLACK);
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
                MUTED,
            );
            toolpath_legend(&painter, rect);
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
        if response.clicked()
            && ui.input(|i| i.modifiers.ctrl)
            && self.snapshot.can_jog()
            && !self.snapshot.status.torch
            && !ui.ctx().egui_wants_keyboard_input()
            && let Some(pointer) = response.interact_pointer_pos()
        {
            self.controller.send(Action::JogTo {
                x: center.x + ((pointer.x - rect.center().x - self.pan.x) / scale) as f64,
                y: center.y - ((pointer.y - rect.center().y - self.pan.y) / scale) as f64,
                speed: self.jog_feed(ui.input(|i| i.modifiers.shift)),
            });
        }
        response.on_hover_text("Ctrl-click: jog to this work position · drag: pan · wheel: zoom");
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
                    Stroke::new(1.0_f32, Color32::from_gray(48)),
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
                    Stroke::new(1.0_f32, Color32::from_gray(48)),
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
            let base_stroke = if block.rapid {
                RAPID_STROKE
            } else {
                FEED_STROKE
            };
            let color = if active {
                Color32::WHITE
            } else if completed {
                Color32::from_rgb(111, 137, 148)
            } else {
                base_stroke.color
            };
            for points in block.path.windows(2) {
                painter.line_segment(
                    [project(points[0]), project(points[1])],
                    Stroke::new(if active { 2.8_f32 } else { base_stroke.width }, color),
                );
            }
        }
        if self.show_hull && self.hull_points.len() >= 2 {
            for i in 0..self.hull_points.len() {
                painter.line_segment(
                    [
                        project(self.hull_points[i]),
                        project(self.hull_points[(i + 1) % self.hull_points.len()]),
                    ],
                    Stroke::new(1.0_f32, Color32::from_gray(210)),
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
            MUTED,
        );
        toolpath_legend(&painter, rect);
    }
    fn code_view(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("G-CODE").strong());
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
            ui.small("Ctrl+scroll: UI size");
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
        let code_width = columns as f32 * CODE_FONT_SIZE * 0.65 + 72.0;
        let height = 22.0;
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
                let (number, source) = if self.show_original {
                    ((row + 1).to_string(), program.source_lines[row].as_str())
                } else {
                    let b = &program.blocks[row];
                    (format!("N{}", b.n), b.code.as_str())
                };
                let (rect, _) = ui.allocate_exact_size(
                    Vec2::new(ui.available_width().max(code_width), height),
                    egui::Sense::hover(),
                );
                if active == Some(row) {
                    ui.painter()
                        .rect_filled(rect, 2.0, Color32::from_rgb(25, 45, 65));
                    ui.painter()
                        .vline(rect.left(), rect.y_range(), Stroke::new(2.0_f32, CYAN));
                }
                ui.painter().text(
                    rect.left_center() + Vec2::new(52.0, 0.0),
                    egui::Align2::RIGHT_CENTER,
                    number,
                    egui::FontId::monospace(CODE_FONT_SIZE),
                    if self.show_original {
                        LINE_NUMBER
                    } else {
                        gcode_color('N')
                    },
                );
                let galley = ui.painter().layout_job(highlight_gcode(source));
                let position = rect.left_center() + Vec2::new(68.0, -galley.size().y / 2.0);
                ui.painter().galley(position, galley, TEXT);
            }
        });
    }
    fn update_ui_zoom(&mut self, ctx: &egui::Context) {
        let delta = ctx.input(|input| input.zoom_delta());
        if delta != 1.0 {
            self.settings.text_size = (self.settings.text_size * delta.powf(0.4)).clamp(8.0, 40.0);
            self.scroll_to_active = true;
            if !self.demo {
                self.settings.save();
            }
        }
        crate::desktop::set_ui_scale(ctx, self.settings.text_size / 13.0);
    }
    fn open_chooser(&mut self) {
        if !self.snapshot.job
            && !self.snapshot.tracing
            && !self.snapshot.transferring
            && self.picker.is_none()
        {
            self.controller.send(Action::CancelJog);
            self.picker_purpose = crate::file_dialog::Purpose::Gcode;
            self.picker = Some(crate::file_dialog::open(
                self.settings.last_dir.clone(),
                self.native_window,
            ));
        }
    }
    fn dialogs(&mut self, root: &mut egui::Ui) {
        let ctx = root.ctx().clone();
        if self.settings_open {
            let mut open = true;
            let mut apply = false;
            egui::Window::new("Settings")
                .open(&mut open)
                .default_width(430.0)
                .show(&ctx, |ui| {
                    ui.add_enabled_ui(!self.demo, |ui| {
                        ui.label("Controller TCP address");
                        ui.text_edit_singleline(&mut self.settings.address);
                        ui.label("Controller HTTP URL (file upload)");
                        ui.text_edit_singleline(&mut self.settings.http);
                    });
                    if ui
                        .add_enabled(
                            !self.snapshot.job
                                && !self.snapshot.tracing
                                && !self.snapshot.transferring,
                            egui::Button::new("Apply settings"),
                        )
                        .clicked()
                    {
                        apply = true;
                    }
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
                    if !self.demo {
                        self.settings.save();
                    }
                    self.settings_open = false;
                }
            }
        }
        if self.console_open {
            self.console_dialog(&ctx);
        }
        if self.files_open {
            self.files_dialog(&ctx);
        }
        if self.log_open {
            let mut open = true;
            egui::Window::new("Application log")
                .open(&mut open)
                .default_size([700.0, 300.0])
                .show(&ctx, |ui| {
                    egui::ScrollArea::both()
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for line in &self.snapshot.log {
                                ui.label(RichText::new(line).monospace());
                            }
                        });
                });
            self.log_open = open;
        }
    }
    fn files_dialog(&mut self, ctx: &egui::Context) {
        let mut open = true;
        egui::Window::new("Controller files")
            .open(&mut open)
            .default_width(560.0)
            .show(ctx, |ui| {
                ui.label(format!("Files on {}", self.settings.http));
                let available = self.snapshot.can_manage_files() && self.picker.is_none();
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(available, egui::Button::new("Refresh"))
                        .clicked()
                    {
                        self.controller.send(Action::ListFiles);
                    }
                    if ui
                        .add_enabled(available, egui::Button::new("Upload file…"))
                        .clicked()
                    {
                        self.picker_purpose = crate::file_dialog::Purpose::Upload;
                        self.picker = Some(crate::file_dialog::choose(
                            self.settings.last_dir.clone(),
                            self.native_window,
                            self.picker_purpose.clone(),
                        ));
                    }
                });
                ui.small(
                    "Uploads keep the local filename and replace an existing file of that name.",
                );
                egui::ScrollArea::vertical()
                    .id_salt("controller_files")
                    .max_height(240.0)
                    .show(ui, |ui| {
                        egui::Grid::new("controller_file_list")
                            .striped(true)
                            .show(ui, |ui| {
                                for file in &self.snapshot.files {
                                    if ui
                                        .selectable_label(
                                            self.remote_filename == file.name,
                                            &file.name,
                                        )
                                        .clicked()
                                    {
                                        self.remote_filename = file.name.clone();
                                    }
                                    ui.label(format!("{} bytes", file.bytes));
                                    ui.end_row();
                                }
                            });
                    });
                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("Filename");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.remote_filename).desired_width(280.0),
                    );
                    if ui
                        .add_enabled(
                            available && !self.remote_filename.is_empty(),
                            egui::Button::new("Download…"),
                        )
                        .clicked()
                    {
                        match crate::protocol::validate_filename(&self.remote_filename) {
                            Ok(()) => {
                                self.picker_purpose = crate::file_dialog::Purpose::Download(
                                    self.remote_filename.clone(),
                                );
                                self.picker = Some(crate::file_dialog::choose(
                                    self.settings.last_dir.clone(),
                                    self.native_window,
                                    self.picker_purpose.clone(),
                                ));
                            }
                            Err(error) => self.error = Some(error.to_string()),
                        }
                    }
                });
                ui.label(&self.snapshot.files_message);
                if !available && self.picker.is_none() && !self.snapshot.transferring {
                    ui.small("Connect with the controller idle or in Alarm to transfer files.");
                }
                ui.small("Configuration uploads take effect after restarting the controller.");
            });
        self.files_open = open;
    }
    fn console_dialog(&mut self, ctx: &egui::Context) {
        if ctx.input(|input| input.key_pressed(egui::Key::Escape)) {
            self.console_open = false;
            self.console_focus = false;
            self.console_submit = false;
            if let Some(id) = self.console_input_id {
                ctx.memory_mut(|memory| memory.surrender_focus(id));
            }
            return;
        }
        let mut open = true;
        egui::Window::new("FluidNC console")
            .open(&mut open)
            .default_size([760.0, 420.0])
            .show(ctx, |ui| {
                ui.checkbox(&mut self.console_status, "Show status reports");
                console_output(ui, &self.snapshot, self.console_status);
                ui.horizontal(|ui| {
                    let id = egui::Id::new("fluidnc_console_command");
                    self.console_input_id = Some(id);
                    let mut output = egui::TextEdit::singleline(&mut self.console_input)
                            .id(id)
                            .desired_width((ui.available_width() - 65.0).max(100.0))
                            .font(egui::TextStyle::Monospace)
                            .hint_text("FluidNC command")
                            .show(ui);
                    let field = output.response;
                    if self.console_focus && ui.is_visible() {
                        field.request_focus();
                        output.state.cursor.set_char_range(Some(egui::text::CCursorRange::one(
                            egui::text::CCursor::new(self.console_input.chars().count()),
                        )));
                        output.state.store(ctx, id);
                        self.console_focus = false;
                    }
                    if field.has_focus() || field.lost_focus() {
                        if ui.input(|i| i.key_pressed(egui::Key::ArrowUp))
                            && self.console_index > 0
                        {
                            if self.console_index == self.console_history.len() {
                                self.console_draft = self.console_input.clone();
                            }
                            self.console_index -= 1;
                            self.console_input = self.console_history[self.console_index].clone();
                        }
                        if ui.input(|i| i.key_pressed(egui::Key::ArrowDown))
                            && self.console_index < self.console_history.len()
                        {
                            self.console_index += 1;
                            self.console_input = self.console_history
                                .get(self.console_index).cloned()
                                .unwrap_or_else(|| self.console_draft.clone());
                        }
                    }
                    let submit = self.console_submit || field.lost_focus()
                        && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    self.console_submit = false;
                    let submit = ui.button("Send").clicked() || submit;
                    if submit && !self.console_input.trim().is_empty() {
                        let command = self.console_input.trim().to_owned();
                        if command == "?" {
                            self.console_status = true;
                        }
                        self.console_history.push(command.clone());
                        if self.console_history.len() > 100 {
                            self.console_history.remove(0);
                        }
                        self.console_index = self.console_history.len();
                        self.console_input.clear();
                        self.console_draft.clear();
                        self.controller.send(Action::Console(command));
                        if !field.has_focus() { field.request_focus(); }
                    }
                });
                ui.small("Esc closes · Enter sends · Up/Down recall · ordinary commands need Idle/Alarm; ? ! ~ work during jobs");
            });
        self.console_open = open;
        if !open && let Some(id) = self.console_input_id {
            ctx.memory_mut(|memory| memory.surrender_focus(id));
        }
    }
}
fn take_shortcut(events: &mut Vec<egui::Event>) -> Option<char> {
    let (index, prefix) = events
        .iter()
        .enumerate()
        .find_map(|(index, event)| match event {
            egui::Event::Key {
                key,
                pressed: true,
                repeat: false,
                modifiers,
                ..
            } if !modifiers.ctrl && !modifiers.alt && !modifiers.command => match key {
                egui::Key::G => Some((index, 'G')),
                egui::Key::M => Some((index, 'M')),
                egui::Key::X => Some((index, 'X')),
                egui::Key::Y => Some((index, 'Y')),
                egui::Key::F => Some((index, 'F')),
                egui::Key::I => Some((index, 'I')),
                egui::Key::O => Some((index, 'O')),
                egui::Key::R => Some((index, 'R')),
                egui::Key::P => Some((index, 'P')),
                egui::Key::S => Some((index, 'S')),
                _ => None,
            },
            _ => None,
        })?;
    events.remove(index);
    // The X11 backend also emits text for the shortcut key. Consume that one
    // character so it is not inserted into the newly focused field.
    if let Some(offset) = events[index..].iter().position(|event| {
        matches!(event, egui::Event::Text(text) if text.eq_ignore_ascii_case(&prefix.to_string()))
    }) {
        events.remove(index + offset);
    }
    Some(prefix)
}
fn duration(seconds: f64) -> String {
    if !seconds.is_finite() {
        return "—".into();
    }
    let s = seconds.max(0.0).round() as u64;
    format!("{:02}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn console_layout_stays_fixed_during_hidden_status_updates() {
        let ctx = egui::Context::default();
        configure_style(&ctx);
        let mut snapshot = Snapshot {
            console_output: Arc::new(
                (0..200)
                    .map(|i| format!("> {i}: {}", "G1X100Y200F600 ".repeat(12)))
                    .collect(),
            ),
            ..Default::default()
        };
        let mut stable = None;
        for frame in 0..40 {
            Arc::make_mut(&mut snapshot.console).push_back(format!("<Run|MPos:{frame},0,0>"));
            let mut geometry = None;
            let output = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        Pos2::ZERO,
                        Vec2::new(640.0, 480.0),
                    )),
                    time: Some(frame as f64 / 30.0),
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show_inside(ui, |ui| {
                        let scroll = console_output(ui, &snapshot, false);
                        geometry =
                            Some((scroll.inner_rect, scroll.content_size, scroll.state.offset));
                    });
                },
            );
            if frame >= 10 {
                let geometry = geometry.unwrap();
                assert!(geometry.1.x > 640.0, "Long lines must scroll horizontally");
                if let Some((expected, shapes)) = &stable {
                    assert_eq!(&geometry, expected);
                    assert_eq!(&output.shapes, shapes);
                } else {
                    stable = Some((geometry, output.shapes));
                }
            }
        }
    }

    #[test]
    fn highlighting_keeps_values_with_words_and_comments_out_of_code() {
        let source = "g01X-2.5 Y+3 F600 (Ø X999 (nested)) M5 ; G0X99";
        let job = highlight_gcode(source);
        assert_eq!(job.text, source);
        for (token, color) in [
            ("g01", gcode_color('G')),
            ("X-2.5", gcode_color('X')),
            ("Y+3", gcode_color('Y')),
            ("F600", gcode_color('F')),
            ("(Ø X999 (nested))", COMMENT),
            ("M5", gcode_color('M')),
            ("; G0X99", COMMENT),
        ] {
            assert!(
                job.sections.iter().any(|section| {
                    job.text[section.byte_range.clone()].trim() == token
                        && section.format.color == color
                }),
                "Incorrect highlight for {token}"
            );
        }
        let unfinished = "X1 (unfinished Ø comment";
        let job = highlight_gcode(unfinished);
        assert_eq!(job.text, unfinished);
        assert_eq!(job.sections.last().unwrap().format.color, COMMENT);
    }
}
