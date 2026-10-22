use crate::{
    gcode::{self, ParseOptions, Program, TimeParts},
    protocol::{self, Link, Status},
};
use anyhow::{Result, ensure};
use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex,
        mpsc::{self, Receiver, Sender},
    },
    thread,
    time::{Duration, Instant},
};

#[derive(Clone, Debug)]
pub enum Action {
    Load {
        name: String,
        text: String,
        pierce: f64,
    },
    Run,
    Pause,
    Resume,
    Stop,
    Unlock,
    Feed(u8),
    Jog {
        x: f64,
        y: f64,
        speed: f64,
    },
    CancelJog,
    SetCoordinate {
        axis: char,
        value: f64,
    },
    Connect {
        address: String,
        http: String,
    },
}

#[derive(Clone)]
pub struct Snapshot {
    pub connected: bool,
    pub status: Status,
    pub ready: bool,
    pub transferring: bool,
    pub initialized: bool,
    pub name: String,
    pub program: Option<Arc<Program>>,
    pub upload: String,
    pub free: Option<String>,
    pub error: Option<String>,
    pub log: VecDeque<String>,
    pub job: bool,
    pub phase: String,
    pub elapsed: f64,
    pub remaining: f64,
    pub coefficient: f64,
    pub progress: f64,
    pub active: Option<usize>,
    pub timing_uncertain: bool,
    pub options: ParseOptions,
    pub line_numbers: bool,
    pub version: String,
}
impl Default for Snapshot {
    fn default() -> Self {
        Self {
            connected: false,
            status: Status::default(),
            ready: false,
            transferring: false,
            initialized: false,
            name: String::new(),
            program: None,
            upload: "Open a G-code file to begin".into(),
            free: None,
            error: None,
            log: VecDeque::new(),
            job: false,
            phase: String::new(),
            elapsed: 0.0,
            remaining: 0.0,
            coefficient: 1.0,
            progress: 0.0,
            active: None,
            timing_uncertain: false,
            options: ParseOptions::default(),
            line_numbers: false,
            version: String::new(),
        }
    }
}
impl Snapshot {
    pub fn can_run(&self) -> bool {
        self.connected
            && self.initialized
            && self.status.fresh()
            && self.status.idle()
            && self.status.position_known
            && self.status.wco_known
            && !self.status.torch
            && self.ready
            && !self.job
            && !self.transferring
    }
    pub fn can_jog(&self) -> bool {
        self.connected
            && self.initialized
            && self.status.fresh()
            && !self.job
            && !self.transferring
            && (self.status.idle() || self.status.state == "Jog")
    }
    fn log(&mut self, text: impl Into<String>) {
        self.log.push_back(text.into());
        while self.log.len() > 200 {
            self.log.pop_front();
        }
    }
}

pub struct Controller {
    tx: Option<Sender<Action>>,
    shared: Arc<Mutex<Snapshot>>,
    handle: Option<thread::JoinHandle<()>>,
}
impl Controller {
    pub fn start(address: String, http: String) -> Self {
        let (tx, rx) = mpsc::channel();
        let shared = Arc::new(Mutex::new(Snapshot::default()));
        let state = shared.clone();
        let handle = thread::spawn(move || Worker::new(address, http, rx, state).run());
        Self {
            tx: Some(tx),
            shared,
            handle: Some(handle),
        }
    }
    pub fn send(&self, action: Action) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(action);
        }
    }
    pub fn snapshot(&self) -> Snapshot {
        self.shared.lock().unwrap().clone()
    }
}
impl Drop for Controller {
    fn drop(&mut self) {
        if let Some(tx) = self.tx.take() {
            let _ = tx.send(Action::CancelJog);
            drop(tx);
        }
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum PauseState {
    None,
    Holding,
    TurningOff { was_lit: bool, sent: Instant },
    Paused { was_lit: bool },
    Restoring { sent: Instant },
    Piercing { until: Instant },
}

#[derive(Default)]
struct Tracker {
    elapsed: f64,
    expected_motion: f64,
    dwell: f64,
    completed: TimeParts,
    fraction: f64,
    last_feed: f64,
    last_rapid: f64,
    resume_dwell: f64,
    last_index: Option<usize>,
    block_fraction: f64,
}
impl Tracker {
    fn sample(&mut self, program: &Program, status: &Status) -> (Option<usize>, f64) {
        let Some(index) = status
            .line
            .and_then(|n| n.checked_sub(1))
            .filter(|i| *i < program.blocks.len())
        else {
            return (None, self.fraction);
        };
        let block = &program.blocks[index];
        let part = block.fraction_after(
            status.wpos,
            if self.last_index == Some(index) {
                self.block_fraction
            } else {
                0.0
            },
        );
        self.last_index = Some(index);
        self.block_fraction = part;
        let completed = block.before.plus(block.time.scaled(part));
        let delta = completed.minus(self.completed);
        self.expected_motion +=
            delta.feed / self.last_feed.max(0.01) + delta.rapid / self.last_rapid.max(0.01);
        self.dwell += delta.dwell;
        if completed.feed + completed.rapid + completed.dwell
            >= self.completed.feed + self.completed.rapid + self.completed.dwell
        {
            self.completed = completed;
        }
        self.last_feed = status.feed_override;
        self.last_rapid = status.rapid_override;
        let total = program.total.feed + program.total.rapid + program.total.dwell;
        self.fraction = ((self.completed.feed + self.completed.rapid + self.completed.dwell)
            / total.max(1e-9))
        .clamp(0.0, 1.0);
        (Some(index), self.fraction)
    }
    fn coefficient(&self) -> f64 {
        if self.expected_motion < 2.0 {
            1.0
        } else {
            ((self.elapsed - self.dwell - self.resume_dwell).max(0.0) / self.expected_motion)
                .clamp(0.1, 10.0)
        }
    }
}

enum Transfer {
    Message(String),
    Done(Result<protocol::FileInfo>),
}
struct Worker {
    address: String,
    http: String,
    rx: Receiver<Action>,
    shared: Arc<Mutex<Snapshot>>,
    s: Snapshot,
    link: Option<Link>,
    queue: VecDeque<String>,
    pending: Option<(String, Instant)>,
    reconnect: Instant,
    poll: Instant,
    transfer: Option<Receiver<Transfer>>,
    need_upload: bool,
    need_verify: bool,
    failed_transfer: bool,
    pause: PauseState,
    tracker: Tracker,
    last_tick: Instant,
    jog: Option<(f64, f64, f64, Instant)>,
    last_jog: Instant,
    inches: bool,
    got_units: bool,
    init_started: bool,
    reconcile_job: bool,
    reconcile_pause: bool,
    preview_dirty: bool,
    lost_at: Option<(Instant, bool)>,
    selected_work_system: bool,
    reset_wait: Option<Instant>,
}
impl Worker {
    fn new(
        address: String,
        http: String,
        rx: Receiver<Action>,
        shared: Arc<Mutex<Snapshot>>,
    ) -> Self {
        let now = Instant::now();
        Self {
            address,
            http,
            rx,
            shared,
            s: Snapshot::default(),
            link: None,
            queue: VecDeque::new(),
            pending: None,
            reconnect: now,
            poll: now,
            transfer: None,
            need_upload: false,
            need_verify: false,
            failed_transfer: false,
            pause: PauseState::None,
            tracker: Tracker {
                last_feed: 1.0,
                last_rapid: 1.0,
                ..Default::default()
            },
            last_tick: now,
            jog: None,
            last_jog: now,
            inches: false,
            got_units: false,
            init_started: false,
            reconcile_job: false,
            reconcile_pause: false,
            preview_dirty: false,
            lost_at: None,
            selected_work_system: false,
            reset_wait: None,
        }
    }
    fn run(mut self) {
        loop {
            let now = Instant::now();
            let dt = now.duration_since(self.last_tick).as_secs_f64();
            self.last_tick = now;
            let actively_piercing = matches!(
                self.pause,
                PauseState::Piercing { .. } | PauseState::Restoring { .. }
            );
            if self.s.job
                && self.s.connected
                && (self.s.phase == "Starting"
                    || actively_piercing
                    || (self.s.status.state == "Run"
                        || self.s.status.state == "Hold:1"
                        || self.s.status.state == "Idle" && self.s.status.file.is_some())
                        && !matches!(self.pause, PauseState::Paused { .. }))
            {
                self.tracker.elapsed += dt;
                if actively_piercing {
                    self.tracker.resume_dwell += dt;
                }
            }
            loop {
                match self.rx.try_recv() {
                    Ok(action) => {
                        if let Err(e) = self.action(action) {
                            self.fail(format!("{e:#}"));
                        }
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => {
                        let _ = self.realtime(&[protocol::JOG_CANCEL]);
                        return;
                    }
                }
            }
            if self.link.is_none() && now >= self.reconnect {
                self.reconnect = now + Duration::from_secs(2);
                match Link::connect(&self.address) {
                    Ok(link) => {
                        self.link = Some(link);
                        self.s.connected = true;
                        self.s.status = Status::default();
                        self.s.status.state = "Connecting".into();
                        self.s.initialized = false;
                        self.got_units = false;
                        self.init_started = false;
                        self.selected_work_system = false;
                        self.queue.clear();
                        self.pending = None;
                        self.queue.push_back("$report_inches".into());
                        self.queue.push_back("$I".into());
                        self.need_verify = self.s.program.is_some() && !self.need_upload;
                        self.preview_dirty = true;
                        self.s.log(format!("Connected to {}", self.address));
                    }
                    Err(_) => {
                        self.s.phase = "Reconnecting…".into();
                    }
                }
            }
            if let Err(e) = self.network() {
                self.disconnected(format!("{e:#}"));
            }
            if let Err(e) = self.advance_pause() {
                self.fail(format!("{e:#}"));
            }
            if let Err(e) = self.advance_jog() {
                self.fail(format!("{e:#}"));
            }
            self.advance_transfer();
            self.s.elapsed = self.tracker.elapsed;
            if !self.s.timing_uncertain {
                self.s.coefficient = self.tracker.coefficient();
            }
            if let Some(p) = &self.s.program {
                self.s.remaining = p.total.minus(self.tracker.completed).seconds(
                    self.s.status.feed_override,
                    self.s.status.rapid_override,
                    self.s.coefficient,
                );
                if let PauseState::Piercing { until } = self.pause {
                    self.s.remaining += until
                        .saturating_duration_since(Instant::now())
                        .as_secs_f64();
                }
                if matches!(self.pause, PauseState::Restoring { .. }) {
                    self.s.remaining += self.s.options.pierce_seconds;
                }
            }
            *self.shared.lock().unwrap() = self.s.clone();
            thread::sleep(Duration::from_millis(10));
        }
    }
    fn fail(&mut self, message: String) {
        self.s.log(format!("Error: {message}"));
        self.s.error = Some(message);
    }
    fn disconnected(&mut self, message: String) {
        if self.s.connected && self.s.job {
            self.lost_at = Some((Instant::now(), self.s.status.state == "Run"));
        }
        self.link = None;
        self.s.connected = false;
        self.s.initialized = false;
        self.s.ready = false;
        self.s.status.state = "Disconnected".into();
        self.s.phase = "Reconnecting…".into();
        self.s.log(message);
        self.queue.clear();
        self.pending = None;
        self.jog = None;
        self.s.status.at = None;
        self.reset_wait = None;
        if self.s.job {
            self.s.timing_uncertain = true;
            self.reconcile_job = true;
        }
        // Never replay a torch toggle, cycle-start, jog or coordinate write after reconnect.
        self.pause = match self.pause {
            PauseState::TurningOff { was_lit, .. } | PauseState::Paused { was_lit } => {
                PauseState::Paused { was_lit }
            }
            PauseState::Restoring { .. } | PauseState::Piercing { .. } => {
                PauseState::Paused { was_lit: true }
            }
            other => other,
        };
        self.reconcile_pause = !matches!(self.pause, PauseState::None);
    }
    fn realtime(&mut self, bytes: &[u8]) -> Result<()> {
        self.link.as_mut().context_link()?.send(bytes)
    }
    fn network(&mut self) -> Result<()> {
        let Some(link) = &mut self.link else {
            return Ok(());
        };
        if self.poll.elapsed() > Duration::from_millis(200) {
            link.send(b"?")?;
            self.poll = Instant::now();
        }
        let lines = link.poll()?;
        for line in lines {
            self.line(&line)?;
        }
        if self
            .reset_wait
            .is_some_and(|t| t.elapsed() > Duration::from_secs(3))
        {
            anyhow::bail!("Reset handshake timed out");
        }
        if self
            .s
            .status
            .at
            .is_some_and(|t| t.elapsed() > Duration::from_secs(4))
        {
            anyhow::bail!("Controller status timed out");
        }
        if self.s.status.at.is_none()
            && self
                .pending
                .as_ref()
                .is_some_and(|(_, t)| t.elapsed() > Duration::from_secs(4))
        {
            anyhow::bail!("Controller handshake timed out");
        }
        if self.pending.as_ref().is_some_and(|(cmd, t)| {
            !cmd.starts_with("$LocalFS/Run=") && t.elapsed() > Duration::from_secs(5)
        }) {
            anyhow::bail!("Controller command timed out; it will not be repeated");
        }
        if self.pending.is_none() {
            if let Some(command) = self.queue.pop_front() {
                self.link.as_mut().unwrap().command(&command)?;
                self.s.log(format!("> {command}"));
                self.pending = Some((command, Instant::now()));
            } else if self.init_started && self.got_units && self.s.status.fresh() {
                if !self.selected_work_system && self.s.status.idle() && !self.s.job {
                    self.selected_work_system = true;
                    self.queue.push_back("G54".into());
                    self.queue.push_back("$G".into());
                    self.s.status.wco_known = false;
                    self.s.status.position_known = false;
                } else {
                    self.s.initialized = true;
                }
                if !self.s.job && self.s.phase == "Reconnecting…" {
                    self.s.phase.clear();
                }
            }
        }
        Ok(())
    }
    fn line(&mut self, line: &str) -> Result<()> {
        if line.starts_with('<') {
            // Status positions cannot be interpreted until Report/Inches is known.
            if !self.got_units {
                return Ok(());
            }
            let previous_state = self.s.status.state.clone();
            self.s.status.update(line, self.inches);
            if self.reconcile_job && self.s.status.at.is_some() {
                if let Some((lost, was_running)) = self.lost_at.take()
                    && was_running
                    && self.s.status.state == "Run"
                {
                    self.tracker.elapsed += lost.elapsed().as_secs_f64();
                }
                if self.s.status.idle() {
                    self.reconcile_job = false;
                    self.s.job = false;
                    self.pause = PauseState::None;
                    self.s.phase = "Job ended while disconnected".into();
                    self.need_verify = self.s.program.is_some();
                    self.reconcile_pause = false;
                }
            }
            if self.reconcile_pause && self.s.status.accessory_at.is_some() {
                self.reconcile_pause = false;
                if self.s.status.held() {
                    let remembered = match self.pause {
                        PauseState::Paused { was_lit } => was_lit,
                        _ => self.s.status.torch,
                    };
                    if self.s.status.torch {
                        self.realtime(&[protocol::TORCH_TOGGLE])?;
                        self.pause = PauseState::TurningOff {
                            was_lit: remembered,
                            sent: Instant::now(),
                        };
                    } else {
                        self.pause = PauseState::Paused {
                            was_lit: remembered,
                        };
                        self.s.phase = "Paused · torch off".into();
                    }
                } else if self.s.status.state == "Run" {
                    self.pause = PauseState::None;
                }
            }
            // A job may finish before the first post-Run poll. Its deferred ack
            // is authoritative; do not infer completion from a stale Idle sample.
            if self.s.status.position_known {
                self.s.options.start = self.s.status.wpos;
                self.s.options.wco = self.s.status.wco;
            }
            if self.s.status.state.starts_with("Hold")
                && matches!(self.pause, PauseState::None)
                && self.s.job
                && !previous_state.starts_with("Hold")
            {
                // A programmed M0/M1 is treated like a UI pause, including torch-off.
                self.pause = PauseState::Holding;
            }
            if self.s.job {
                if let Some(p) = &self.s.program {
                    let (active, progress) = self.tracker.sample(p, &self.s.status);
                    if active.is_some() {
                        self.s.active = active;
                    }
                    self.s.progress = progress;
                }
                if self
                    .s
                    .status
                    .file
                    .as_ref()
                    .is_some_and(|f| !f.ends_with("/plasma.nc") && f != "plasma.nc")
                {
                    self.s.timing_uncertain = true;
                    self.fail(
                        "A different controller file is running; local line mapping is unavailable"
                            .into(),
                    );
                }
            }
            return Ok(());
        }
        self.s.log(line);
        if line.starts_with("Grbl ") && (self.s.initialized || self.reset_wait.is_some()) {
            self.reset_wait = None;
            self.s.ready = false;
            self.s.job = false;
            self.pause = PauseState::None;
            self.queue.clear();
            self.pending = None;
            self.s.initialized = false;
            self.init_started = false;
            self.selected_work_system = false;
            self.got_units = false;
            self.queue.push_back("$report_inches".into());
            self.queue.push_back("$I".into());
            self.need_verify = self.s.program.is_some();
        }
        if line.contains("FluidNC") {
            self.s.version = line.into();
        }
        if let Some(value) = line.strip_prefix("$13=") {
            self.inches = value.trim() == "1";
            self.got_units = true;
        }
        if let Some(value) = line.strip_prefix("$/report_inches=") {
            self.inches = value.trim() == "true";
            self.got_units = true;
        }
        if let Some(value) = line.strip_prefix("$/use_line_numbers=") {
            self.s.line_numbers = value.trim() == "true";
        }
        for (axis, key) in [
            ('x', "$/axes/x/max_rate_mm_per_min="),
            ('y', "$/axes/y/max_rate_mm_per_min="),
            ('z', "$/axes/z/max_rate_mm_per_min="),
        ] {
            if let Some(v) = line
                .strip_prefix(key)
                .and_then(|v| v.parse::<f64>().ok())
                .filter(|v| *v > 0.0)
            {
                match axis {
                    'x' => self.s.options.rapid.x = v,
                    'y' => self.s.options.rapid.y = v,
                    _ => self.s.options.rapid.z = v,
                }
            }
        }
        if let Some(modal) = line.strip_prefix("[GC:").and_then(|s| s.strip_suffix(']')) {
            for word in modal.split_whitespace() {
                match word {
                    "G20" => self.s.options.initial_mm = false,
                    "G21" => self.s.options.initial_mm = true,
                    "G90" => self.s.options.initial_absolute = true,
                    "G91" => self.s.options.initial_absolute = false,
                    "G0" => self.s.options.initial_motion = 0,
                    "G1" => self.s.options.initial_motion = 1,
                    "G2" => self.s.options.initial_motion = 2,
                    "G3" => self.s.options.initial_motion = 3,
                    "M3" => self.s.options.initial_spindle = true,
                    "M5" => self.s.options.initial_spindle = false,
                    "G17" => self.s.options.initial_xy_plane = true,
                    "G18" | "G19" => self.s.options.initial_xy_plane = false,
                    "G94" => self.s.options.initial_feed_per_minute = true,
                    "G93" => self.s.options.initial_feed_per_minute = false,
                    _ => {}
                }
                if let Some(f) = word.strip_prefix('F').and_then(|s| s.parse::<f64>().ok()) {
                    // FluidNC 4.1.1 reports its internal mm/min feed in $G,
                    // even when the modal units are G20 (Report.cpp).
                    self.s.options.initial_feed = f;
                }
            }
            self.s.options.initial_speed = modal
                .split_whitespace()
                .find_map(|w| w.strip_prefix('S')?.parse().ok())
                .unwrap_or(0.0);
        }
        if (line == "ok" || line.starts_with("error:"))
            && let Some((cmd, _)) = self.pending.take()
        {
            if line.starts_with("error:") {
                self.fail(format!("{cmd}: {line}"));
                if cmd.starts_with("$LocalFS/Run=") {
                    self.realtime(&[protocol::RESET])?;
                    self.s.job = false;
                }
                if cmd.starts_with("G10") {
                    self.s.status.position_known = false;
                }
            } else if cmd.starts_with("$LocalFS/Run=") {
                self.finish();
            }
            if cmd == "$I" && !self.init_started {
                self.init_started = true;
                for cmd in [
                    "$G",
                    "$use_line_numbers",
                    "$axes/x/max_rate_mm_per_min",
                    "$axes/y/max_rate_mm_per_min",
                    "$axes/z/max_rate_mm_per_min",
                ] {
                    self.queue.push_back(cmd.into());
                }
            }
        }
        if line.starts_with("ALARM:") {
            self.fail(line.into());
            self.s.job = false;
            self.pause = PauseState::None;
        }
        Ok(())
    }
    fn finish(&mut self) {
        self.s.job = false;
        self.pause = PauseState::None;
        self.s.phase = "Completed".into();
        self.s.progress = 1.0;
        if let Some(p) = &self.s.program {
            self.tracker.completed = p.total;
        }
        self.s.log("Job completed");
        self.queue.push_back("$G".into());
    }
    fn action(&mut self, action: Action) -> Result<()> {
        match action {
            Action::Load { name, text, pierce } => {
                ensure!(
                    !self.s.job && self.transfer.is_none(),
                    "Cannot replace a running, paused or uploading job"
                );
                ensure!(
                    !self.s.connected || !self.s.initialized || self.s.status.idle(),
                    "Wait until the controller is idle before opening a file"
                );
                let opts = ParseOptions {
                    pierce_seconds: pierce,
                    ..self.s.options
                };
                let p = gcode::parse(&text, opts)?;
                self.s.options.pierce_seconds = pierce;
                self.s.name = name;
                self.s.program = Some(Arc::new(p));
                self.s.ready = false;
                self.need_upload = true;
                self.failed_transfer = false;
                self.s.error = None;
                self.s.upload = "Awaiting connection / idle controller".into();
                self.s.phase.clear();
                self.tracker = Tracker {
                    last_feed: 1.0,
                    last_rapid: 1.0,
                    ..Default::default()
                };
                self.s.active = None;
                self.s.progress = 0.0;
                self.s.timing_uncertain = false;
                self.preview_dirty = true;
            }
            Action::Run => {
                ensure!(
                    self.s.can_run(),
                    "The current file must be verified and the controller idle before Run"
                );
                ensure!(
                    self.pending.is_none() && self.queue.is_empty(),
                    "Wait for the current controller command to finish"
                );
                // Rebase the preview and timing after jogging/setting the work origin.
                let original = &self.s.program.as_ref().unwrap().original;
                let parsed = gcode::parse(original, self.s.options)?;
                if parsed.uploaded != self.s.program.as_ref().unwrap().uploaded {
                    self.s.program = Some(Arc::new(parsed));
                    self.s.ready = false;
                    self.need_upload = true;
                    self.failed_transfer = false;
                    anyhow::bail!(
                        "Controller modes changed the processed file; uploading the updated job before Run"
                    );
                }
                self.s.program = Some(Arc::new(parsed));
                self.realtime(&[0x99])?; // Relay must not inherit an old spindle-speed override.
                self.queue.push_back("$LocalFS/Run=plasma.nc".into());
                self.s.job = true;
                self.s.phase = "Starting".into();
                self.s.error = None;
                self.tracker = Tracker {
                    last_feed: self.s.status.feed_override,
                    last_rapid: self.s.status.rapid_override,
                    ..Default::default()
                };
                self.s.active = None;
                self.s.timing_uncertain = false;
                self.pause = PauseState::None;
            }
            Action::Pause => {
                ensure!(self.s.connected && self.s.job, "No connected job to pause");
                self.realtime(&[protocol::HOLD])?;
                self.pause = PauseState::Holding;
                self.s.phase = "Holding motion…".into();
            }
            Action::Resume => {
                ensure!(
                    self.s.connected && self.s.status.held(),
                    "Wait for motion to be fully held"
                );
                match self.pause {
                    PauseState::Paused { was_lit: true } => {
                        self.realtime(&[protocol::TORCH_TOGGLE])?;
                        self.pause = PauseState::Restoring {
                            sent: Instant::now(),
                        };
                        self.s.phase = "Restoring torch…".into();
                    }
                    PauseState::Paused { was_lit: false } => {
                        self.realtime(&[protocol::RESUME])?;
                        self.pause = PauseState::None;
                        self.s.phase = "Running".into();
                    }
                    _ => {
                        anyhow::bail!("Wait until the torch-off state is confirmed before resuming")
                    }
                }
            }
            Action::Stop => {
                self.realtime(&[protocol::RESET])?;
                self.s.job = false;
                self.pause = PauseState::None;
                self.jog = None;
                self.s.ready = false;
                self.queue.clear();
                self.pending = None;
                self.s.initialized = false;
                self.init_started = false;
                self.selected_work_system = false;
                self.got_units = false;
                self.reset_wait = Some(Instant::now());
                self.s.phase = "Stopped".into();
                self.need_verify = self.s.program.is_some();
            }
            Action::Unlock => {
                ensure!(
                    self.s.connected && !self.s.job,
                    "Cannot unlock during a job"
                );
                self.queue.push_back("$X".into());
            }
            Action::Feed(byte) => {
                ensure!(
                    [0x90, 0x91, 0x92, 0x93, 0x94].contains(&byte),
                    "Invalid override"
                );
                self.realtime(&[byte])?;
            }
            Action::Jog { x, y, speed } => {
                ensure!(
                    self.s.can_jog() && !self.s.status.torch,
                    "Jogging requires an idle machine and torch off"
                );
                ensure!(
                    speed.is_finite()
                        && (1.0..=3000.0).contains(&speed)
                        && x.is_finite()
                        && y.is_finite(),
                    "Invalid jog speed/direction"
                );
                self.jog = Some((
                    x.clamp(-1.0, 1.0),
                    y.clamp(-1.0, 1.0),
                    speed,
                    Instant::now(),
                ));
                self.preview_dirty = true;
            }
            Action::CancelJog => {
                if self.jog.take().is_some() || self.s.status.state == "Jog" {
                    self.queue.retain(|c| !c.starts_with("$J="));
                    self.realtime(&[protocol::JOG_CANCEL])?;
                }
            }
            Action::SetCoordinate { axis, value } => {
                ensure!(
                    self.s.connected
                        && self.s.initialized
                        && self.s.status.fresh()
                        && self.s.status.idle()
                        && !self.s.job,
                    "Coordinates can only be changed while idle"
                );
                ensure!(
                    !self.s.transferring,
                    "Wait for upload verification before changing coordinates"
                );
                ensure!(
                    matches!(axis, 'X' | 'Y') && value.is_finite(),
                    "Invalid coordinate"
                );
                // G10 L20 sets the work offset at the current physical position. Explicit
                // G21 avoids interpreting a displayed millimetre value as inches.
                self.queue.push_back("G21G54".into());
                self.queue.push_back(format!("G10L20P1{axis}{value:.6}"));
                self.queue.push_back("$G".into());
                self.s.status.wco_known = false;
                self.s.status.position_known = false;
                self.preview_dirty = true;
            }
            Action::Connect { address, http } => {
                ensure!(
                    !self.s.job && self.transfer.is_none(),
                    "Cannot change controller during a job or upload"
                );
                ensure!(
                    !address.contains(['\n', '\r']) && http.starts_with("http://"),
                    "Use a TCP host:port and an http:// URL"
                );
                self.address = address;
                self.http = http.trim_end_matches('/').into();
                self.disconnected("Connection settings changed".into());
                self.reconnect = Instant::now();
            }
        }
        Ok(())
    }
    fn advance_pause(&mut self) -> Result<()> {
        if !self.s.connected {
            return Ok(());
        }
        match self.pause {
            PauseState::Holding if self.s.status.state.starts_with("Hold") => {
                let was_lit = self.s.status.torch;
                if was_lit {
                    self.realtime(&[protocol::TORCH_TOGGLE])?;
                    self.pause = PauseState::TurningOff {
                        was_lit,
                        sent: Instant::now(),
                    };
                    self.s.phase = "Turning torch off…".into();
                } else if self.s.status.held() {
                    self.pause = PauseState::Paused { was_lit: false };
                    self.s.phase = "Paused · torch off".into();
                }
            }
            PauseState::TurningOff { was_lit, sent } => {
                if !self.s.status.torch
                    && self.s.status.held()
                    && self.s.status.accessory_at.is_some_and(|t| t > sent)
                {
                    self.pause = PauseState::Paused { was_lit };
                    self.s.phase = "Paused · torch off".into();
                } else if sent.elapsed() > Duration::from_secs(5) {
                    self.realtime(&[protocol::RESET])?;
                    self.s.job = false;
                    self.pause = PauseState::None;
                    anyhow::bail!("Torch-off confirmation timed out; job aborted");
                }
            }
            PauseState::Restoring { sent } => {
                if self.s.status.torch && self.s.status.accessory_at.is_some_and(|t| t > sent) {
                    self.pause = PauseState::Piercing {
                        until: Instant::now()
                            + Duration::from_secs_f64(self.s.options.pierce_seconds),
                    };
                    self.s.phase = "Piercing before resume…".into();
                } else if sent.elapsed() > Duration::from_secs(5) {
                    self.realtime(&[protocol::RESET])?;
                    self.s.job = false;
                    self.pause = PauseState::None;
                    anyhow::bail!("Torch restore confirmation timed out; job aborted");
                }
            }
            PauseState::Piercing { until } if Instant::now() >= until => {
                self.realtime(&[protocol::RESUME])?;
                self.pause = PauseState::None;
                self.s.phase = "Running".into();
            }
            _ => {}
        }
        if matches!(self.pause, PauseState::None) && self.s.job && self.s.status.state == "Run" {
            self.s.phase = "Running".into();
        }
        Ok(())
    }
    fn advance_jog(&mut self) -> Result<()> {
        if let Some((x, y, speed, heartbeat)) = self.jog {
            if heartbeat.elapsed() > Duration::from_millis(250) || !self.s.can_jog() {
                self.jog = None;
                self.queue.retain(|c| !c.starts_with("$J="));
                self.realtime(&[protocol::JOG_CANCEL])?;
            } else if self.pending.is_none()
                && self.queue.is_empty()
                && self.last_jog.elapsed() > Duration::from_millis(100)
            {
                // Short commands plus a UI heartbeat bound motion if the UI stalls.
                let length = (x * x + y * y).sqrt().max(1.0);
                let distance = speed / 60.0 * 0.12;
                self.queue.push_back(format!(
                    "$J=G21G91X{:.5}Y{:.5}F{speed:.1}",
                    x / length * distance,
                    y / length * distance
                ));
                self.last_jog = Instant::now();
            }
        }
        Ok(())
    }
    fn advance_transfer(&mut self) {
        if self.preview_dirty
            && self.s.initialized
            && self.s.status.idle()
            && self.s.status.position_known
            && !self.s.job
            && self.pending.is_none()
            && self.queue.is_empty()
            && self.transfer.is_none()
        {
            self.preview_dirty = false;
            if let Some(program) = &self.s.program {
                match gcode::parse(&program.original, self.s.options) {
                    Ok(parsed) => {
                        if parsed.uploaded != program.uploaded {
                            self.need_upload = true;
                            self.s.ready = false;
                        }
                        self.s.program = Some(Arc::new(parsed));
                    }
                    Err(e) => {
                        self.s.ready = false;
                        self.failed_transfer = true;
                        self.fail(format!("{e:#}"));
                    }
                }
            }
        }
        if let Some(rx) = &self.transfer {
            let events: Vec<_> = rx.try_iter().collect();
            for event in events {
                match event {
                    Transfer::Message(message) => self.s.upload = message,
                    Transfer::Done(result) => {
                        self.transfer = None;
                        self.s.transferring = false;
                        match result {
                            Ok(info) => {
                                self.s.ready = self.s.connected;
                                self.need_upload = false;
                                self.need_verify = false;
                                self.s.free = info.free;
                                self.s.upload = format!(
                                    "Verified plasma.nc · {} bytes · contents match",
                                    info.bytes
                                );
                                self.s.log(self.s.upload.clone());
                            }
                            Err(e) => {
                                self.s.ready = false;
                                self.failed_transfer = true;
                                self.s.upload =
                                    "Upload / verification failed — reopen the file to retry"
                                        .into();
                                self.fail(format!("{e:#}"));
                            }
                        }
                    }
                }
            }
        }
        if self.transfer.is_none()
            && !self.failed_transfer
            && (self.need_upload || self.need_verify)
            && self.s.connected
            && self.s.initialized
            && self.s.status.fresh()
            && self.s.status.idle()
            && !self.s.job
            && self.pending.is_none()
            && self.queue.is_empty()
            && self.s.status.position_known
            && self.s.status.wco_known
        {
            let Some(program) = self.s.program.clone() else {
                return;
            };
            let base = self.http.clone();
            let upload = self.need_upload;
            let (tx, rx) = mpsc::channel();
            self.transfer = Some(rx);
            self.s.transferring = true;
            self.s.ready = false;
            thread::spawn(move || {
                let result = if upload {
                    protocol::upload(&base, &program.uploaded, |message| {
                        let _ = tx.send(Transfer::Message(message.into()));
                    })
                } else {
                    protocol::verify(&base, &program.uploaded)
                };
                let _ = tx.send(Transfer::Done(result));
            });
        }
    }
}

trait LinkOption<'a> {
    fn context_link(self) -> Result<&'a mut Link>;
}
impl<'a> LinkOption<'a> for Option<&'a mut Link> {
    fn context_link(self) -> Result<&'a mut Link> {
        self.ok_or_else(|| anyhow::anyhow!("Controller is disconnected"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::Point;
    #[test]
    fn fluidnc_modal_feed_is_already_mm_per_minute_in_inch_mode() {
        let (_tx, rx) = mpsc::channel();
        let mut worker = Worker::new(
            "127.0.0.1:1".into(),
            "http://127.0.0.1:1".into(),
            rx,
            Arc::new(Mutex::new(Snapshot::default())),
        );
        worker
            .line("[GC:G1 G54 G17 G20 G90 G94 M5 M9 T0 F60 S0]")
            .unwrap();
        assert!(!worker.s.options.initial_mm);
        assert_eq!(worker.s.options.initial_feed, 60.0);
        let p = gcode::parse("X1", worker.s.options).unwrap();
        assert!((p.total.feed - 25.4).abs() < 1e-9);
    }
    #[test]
    fn tracker_accounts_for_historical_override() {
        let p = gcode::parse("G1X100F60\nG1X200", ParseOptions::default()).unwrap();
        let mut tracker = Tracker {
            last_feed: 2.0,
            last_rapid: 1.0,
            ..Default::default()
        };
        let mut status = Status {
            line: Some(2),
            wpos: Point {
                x: 100.0,
                ..Default::default()
            },
            feed_override: 1.0,
            ..Default::default()
        };
        tracker.elapsed = 50.0;
        tracker.sample(&p, &status);
        assert_eq!(tracker.expected_motion, 50.0);
        assert_eq!(tracker.coefficient(), 1.0);
        assert_eq!(
            p.total
                .minus(tracker.completed)
                .seconds(1.0, 1.0, tracker.coefficient()),
            100.0
        );
        status.wpos.x = 150.0;
        tracker.elapsed = 100.0;
        tracker.sample(&p, &status);
        assert_eq!(tracker.coefficient(), 1.0);
    }
}
