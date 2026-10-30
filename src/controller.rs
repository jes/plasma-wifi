use crate::{
    coordinates::Coordinates,
    gcode::{self, ParseOptions, Point, Program, TimeParts},
    protocol::{self, Link, Status},
};
use anyhow::{Result, ensure};
use std::{
    collections::VecDeque,
    path::PathBuf,
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
    },
    Run,
    Pause,
    Resume,
    Stop,
    Unlock,
    Console(String),
    TraceHull {
        speed: f64,
    },
    CancelTrace,
    Feed(u8),
    Jog {
        x: f64,
        y: f64,
        speed: f64,
        step: f64,
    },
    FinishJog,
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
    pub console: VecDeque<String>,
    pub alarm: Option<String>,
    pub tracing: bool,
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
            console: VecDeque::new(),
            alarm: None,
            tracing: false,
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
            && !self.tracing
    }
    pub fn can_jog(&self) -> bool {
        self.connected
            && self.initialized
            && self.status.fresh()
            && self.status.position_known
            && self.status.wco_known
            && !self.job
            && !self.transferring
            && !self.tracing
            && (self.status.idle() || self.status.state == "Jog")
    }
    fn log(&mut self, text: impl Into<String>) {
        self.log.push_back(text.into());
        while self.log.len() > 200 {
            self.log.pop_front();
        }
    }
    fn console(&mut self, text: impl Into<String>) {
        self.console.push_back(text.into());
        while self.console.len() > 1000 {
            self.console.pop_front();
        }
    }
}

#[derive(Clone, Debug)]
struct QueuedCommand {
    text: String,
    manual: bool,
}
impl From<String> for QueuedCommand {
    fn from(text: String) -> Self {
        Self {
            text,
            manual: false,
        }
    }
}
impl From<&str> for QueuedCommand {
    fn from(text: &str) -> Self {
        text.to_owned().into()
    }
}
impl std::ops::Deref for QueuedCommand {
    type Target = str;
    fn deref(&self) -> &str {
        &self.text
    }
}
impl std::fmt::Display for QueuedCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.text.fmt(f)
    }
}
impl PartialEq<&str> for QueuedCommand {
    fn eq(&self, other: &&str) -> bool {
        self.text == *other
    }
}

struct Trace {
    points: Vec<Point>,
    next: usize,
    sent: Option<Instant>,
    speed: f64,
}

struct ManualJog {
    x: f64,
    y: f64,
    speed: f64,
    step: f64,
    target: Point,
    heartbeat: Instant,
    extended: Instant,
    sent: bool,
    released: bool,
}

pub struct Controller {
    tx: Option<Sender<Action>>,
    shared: Arc<Mutex<Snapshot>>,
    handle: Option<thread::JoinHandle<()>>,
}
impl Controller {
    pub fn start(address: String, http: String, coordinates_path: Option<PathBuf>) -> Self {
        let (tx, rx) = mpsc::channel();
        let shared = Arc::new(Mutex::new(Snapshot::default()));
        let state = shared.clone();
        let handle = thread::spawn(move || {
            let mut worker = Worker::new(address, http, rx, state);
            if let Some(path) = coordinates_path {
                match Coordinates::load(path) {
                    Ok(coordinates) => worker.coordinates = coordinates,
                    Err(e) => worker.fail(format!("Cannot load saved coordinates: {e:#}")),
                }
            }
            worker.run();
        });
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
    restore_time: f64,
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
            ((self.elapsed - self.dwell - self.restore_time).max(0.0) / self.expected_motion)
                .clamp(0.1, 10.0)
        }
    }
}

enum Transfer {
    Message(String),
    Done(Result<protocol::FileInfo>),
}
#[derive(Clone, Copy, Debug)]
enum CoordinateRecovery {
    Pending,
    Writing,
    Verifying(Instant),
    Ready,
    Failed,
}
struct Worker {
    address: String,
    http: String,
    rx: Receiver<Action>,
    shared: Arc<Mutex<Snapshot>>,
    s: Snapshot,
    link: Option<Link>,
    queue: VecDeque<QueuedCommand>,
    pending: Option<(QueuedCommand, Instant)>,
    reconnect: Instant,
    poll: Instant,
    transfer: Option<Receiver<Transfer>>,
    need_upload: bool,
    need_verify: bool,
    failed_transfer: bool,
    pause: PauseState,
    tracker: Tracker,
    last_tick: Instant,
    jog: Option<ManualJog>,
    inches: bool,
    got_units: bool,
    init_started: bool,
    reconcile_job: bool,
    reconcile_pause: bool,
    preview_dirty: bool,
    lost_at: Option<(Instant, bool)>,
    selected_work_system: bool,
    reset_wait: Option<Instant>,
    auto_reports: bool,
    stopping: Option<Instant>,
    trace: Option<Trace>,
    cancel_trace_on_connect: bool,
    coordinates: Coordinates,
    coordinate_recovery: CoordinateRecovery,
    checkpoint_error: bool,
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
            inches: false,
            got_units: false,
            init_started: false,
            reconcile_job: false,
            reconcile_pause: false,
            preview_dirty: false,
            lost_at: None,
            selected_work_system: false,
            reset_wait: None,
            auto_reports: false,
            stopping: None,
            trace: None,
            cancel_trace_on_connect: false,
            coordinates: Coordinates::default(),
            coordinate_recovery: CoordinateRecovery::Pending,
            checkpoint_error: false,
        }
    }
    fn run(mut self) {
        loop {
            let now = Instant::now();
            let dt = now.duration_since(self.last_tick).as_secs_f64();
            self.last_tick = now;
            let restoring_torch = matches!(self.pause, PauseState::Restoring { .. });
            if self.s.job
                && self.s.connected
                && (self.s.phase == "Starting"
                    || restoring_torch
                    || (self.s.status.state == "Run"
                        || self.s.status.state == "Hold:1"
                        || self.s.status.state == "Idle" && self.s.status.file.is_some())
                        && !matches!(self.pause, PauseState::Paused { .. }))
            {
                self.tracker.elapsed += dt;
                if restoring_torch {
                    self.tracker.restore_time += dt;
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
                        self.checkpoint_coordinates();
                        if let Err(e) = self.coordinates.flush() {
                            eprintln!("Cannot save coordinates: {e:#}");
                        }
                        return;
                    }
                }
            }
            if self.link.is_none() && now >= self.reconnect {
                self.reconnect = now + Duration::from_secs(2);
                match Link::connect(&self.address) {
                    Ok(link) => {
                        self.link = Some(link);
                        if self.cancel_trace_on_connect {
                            let _ = self.realtime(&[protocol::JOG_CANCEL]);
                            self.cancel_trace_on_connect = false;
                        }
                        self.s.connected = true;
                        self.s.status = Status::default();
                        self.s.status.state = "Connecting".into();
                        self.s.initialized = false;
                        self.got_units = false;
                        self.init_started = false;
                        self.selected_work_system = false;
                        self.coordinate_recovery = CoordinateRecovery::Pending;
                        self.queue.clear();
                        self.pending = None;
                        self.queue.push_back("$report_inches".into());
                        self.queue.push_back("$Report/Interval=100".into());
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
            if let Err(e) = self.advance_stop() {
                self.fail(format!("{e:#}"));
            }
            if let Err(e) = self.advance_trace() {
                self.fail(format!("{e:#}"));
            }
            if let Err(e) = self.advance_jog() {
                self.fail(format!("{e:#}"));
            }
            self.advance_transfer();
            self.checkpoint_coordinates();
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
        if self.s.tracing {
            let _ = self.realtime(&[protocol::JOG_CANCEL]);
            self.cancel_trace_on_connect = true;
            self.s
                .log("Hull trace cancelled by disconnection; it will not restart");
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
        self.trace = None;
        self.s.tracing = false;
        self.stopping = None;
        self.auto_reports = false;
        self.s.status.at = None;
        self.reset_wait = None;
        if self.s.job {
            self.s.timing_uncertain = true;
            self.reconcile_job = true;
        }
        // Never replay a torch toggle, cycle-start, jog or queued manual coordinate edit.
        // Coordinate recovery uses only the last acknowledged/reported position.
        self.pause = match self.pause {
            PauseState::TurningOff { was_lit, .. } | PauseState::Paused { was_lit } => {
                PauseState::Paused { was_lit }
            }
            PauseState::Restoring { .. } => PauseState::Paused { was_lit: true },
            other => other,
        };
        self.reconcile_pause = !matches!(self.pause, PauseState::None);
    }
    fn realtime(&mut self, bytes: &[u8]) -> Result<()> {
        self.link.as_mut().context_link()?.send(bytes)?;
        for &byte in bytes {
            if byte != b'?' {
                self.s.console(match byte {
                    protocol::HOLD => "> ! (feed hold)".into(),
                    protocol::RESUME => "> ~ (cycle start)".into(),
                    protocol::RESET => "> 0x18 (soft reset)".into(),
                    protocol::JOG_CANCEL => "> 0x85 (cancel jog)".into(),
                    protocol::TORCH_TOGGLE => "> 0x9e (torch override)".into(),
                    _ => format!("> 0x{byte:02x} (realtime override)"),
                });
            }
        }
        Ok(())
    }
    fn network(&mut self) -> Result<()> {
        let Some(link) = &mut self.link else {
            return Ok(());
        };
        if self.poll.elapsed() > Duration::from_millis(if self.auto_reports { 1000 } else { 100 }) {
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
            !cmd.manual && !cmd.starts_with("$LocalFS/Run=") && t.elapsed() > Duration::from_secs(5)
        }) {
            anyhow::bail!("Controller command timed out; it will not be repeated");
        }
        if self.pending.is_none() {
            if let Some(command) = self.queue.pop_front() {
                // An offset-change auto report can arrive before the command's ok.
                // Invalidate before sending so that report remains usable.
                if command.starts_with("G10") || command == "G54" || command == "G21G54" {
                    self.s.status.wco_known = false;
                    self.s.status.position_known = false;
                }
                self.link.as_mut().unwrap().command(&command)?;
                self.s.console(format!("> {command}"));
                self.pending = Some((command, Instant::now()));
            } else if self.init_started && self.got_units && self.s.status.fresh() {
                if !self.selected_work_system && self.s.status.idle() && !self.s.job {
                    self.selected_work_system = true;
                    self.queue.push_back("G54".into());
                    self.queue.push_back("$G".into());
                    self.s.status.wco_known = false;
                    self.s.status.position_known = false;
                } else if self.recover_coordinates()? {
                    self.s.initialized = true;
                }
                if !self.s.job && self.s.phase == "Reconnecting…" {
                    self.s.phase.clear();
                }
            }
        }
        Ok(())
    }
    fn coordinate_tolerance(&self) -> f64 {
        // Report/Inches has a coarser effective resolution after conversion to mm.
        if self.inches { 0.004 } else { 0.001 }
    }
    fn recover_coordinates(&mut self) -> Result<bool> {
        match self.coordinate_recovery {
            CoordinateRecovery::Ready => return Ok(true),
            CoordinateRecovery::Failed => return Ok(false),
            CoordinateRecovery::Writing => {
                // Keep the WCO reported since the offset write. FluidNC sends it
                // on change, then omits it from most Idle reports. Only request
                // a fresh position after all writes have been acknowledged.
                self.s.status.position_known = false;
                self.coordinate_recovery = CoordinateRecovery::Verifying(Instant::now());
                self.realtime(b"?")?;
                return Ok(false);
            }
            CoordinateRecovery::Verifying(started) => {
                if self.s.status.position_known && self.s.status.wco_known {
                    let target = self.coordinates.get(&self.address).unwrap();
                    if self.s.status.wpos.distance(target) <= self.coordinate_tolerance() {
                        self.coordinate_recovery = CoordinateRecovery::Ready;
                        self.s.phase.clear();
                        self.s
                            .log("Work coordinates restored; toolhead assumed unmoved");
                        return Ok(true);
                    }
                }
                if started.elapsed() > Duration::from_secs(3) {
                    self.coordinate_recovery = CoordinateRecovery::Failed;
                    self.s.phase = "Coordinate restoration failed".into();
                    self.fail(
                        "Coordinate restoration could not be verified; reconnect to retry".into(),
                    );
                }
                return Ok(false);
            }
            CoordinateRecovery::Pending => {}
        }
        // A live job/jog can move during an outage. Its coordinates remain authoritative;
        // do not defer a stale restoration until the motion finishes.
        if self.s.status.state == "Run"
            || self.s.status.state == "Jog"
            || self.s.status.state.starts_with("Hold")
            || self.s.status.file.is_some()
        {
            self.coordinate_recovery = CoordinateRecovery::Ready;
            self.s
                .log("Controller is active; keeping its live work coordinates");
            return Ok(true);
        }
        let Some(target) = self.coordinates.get(&self.address) else {
            self.coordinate_recovery = CoordinateRecovery::Ready;
            return Ok(true);
        };
        // Unlock is available while waiting in Alarm. No automatic unlock or motion.
        if !self.selected_work_system
            || !self.s.status.idle()
            || !self.s.status.position_known
            || !self.s.status.wco_known
            || self.s.status.accessory_at.is_none()
            || self.s.status.torch
        {
            return Ok(false);
        }
        if self.s.status.wpos.distance(target) <= self.coordinate_tolerance() {
            self.coordinate_recovery = CoordinateRecovery::Ready;
            return Ok(true);
        }
        self.s.initialized = false;
        self.s.phase = "Restoring work coordinates…".into();
        self.coordinate_recovery = CoordinateRecovery::Writing;
        self.queue.push_back("G21".into());
        self.queue
            .push_back(format!("G10L20P1X{:.6}Y{:.6}Z{:.6}", target.x, target.y, target.z).into());
        if !self.s.options.initial_mm {
            self.queue.push_back("G20".into());
        }
        self.queue.push_back("$G".into());
        self.preview_dirty = true;
        Ok(false)
    }
    fn checkpoint_coordinates(&mut self) {
        let can_checkpoint = self.s.connected
            && self.s.initialized
            && matches!(self.coordinate_recovery, CoordinateRecovery::Ready)
            && self.s.status.fresh()
            && self.s.status.position_known
            && self.s.status.wco_known
            && self
                .pending
                .as_ref()
                .is_none_or(|(cmd, _)| cmd.starts_with("$LocalFS/Run=") || cmd.starts_with("$J="))
            && self.queue.is_empty();
        if !can_checkpoint {
            return;
        }
        if let Err(e) = self.coordinates.record(&self.address, self.s.status.wpos)
            && !self.checkpoint_error
        {
            self.fail(format!(
                "Cannot save coordinates across app restarts: {e:#}"
            ));
            self.checkpoint_error = true;
        }
    }
    fn line(&mut self, line: &str) -> Result<()> {
        self.s.console(line);
        if line.starts_with('<') {
            // Status positions cannot be interpreted until Report/Inches is known.
            if !self.got_units {
                return Ok(());
            }
            let previous_state = self.s.status.state.clone();
            self.s.status.update(line, self.inches);
            if !self.s.status.state.contains("Alarm") {
                self.s.alarm = None;
            }
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
        if line.starts_with("Grbl ")
            && (self.s.initialized
                || self.reset_wait.is_some()
                || self.init_started
                || self.got_units)
        {
            self.trace = None;
            self.s.tracing = false;
            self.stopping = None;
            self.reset_wait = None;
            self.s.ready = false;
            self.s.job = false;
            self.pause = PauseState::None;
            self.queue.clear();
            self.pending = None;
            self.s.initialized = false;
            self.init_started = false;
            self.selected_work_system = false;
            self.coordinate_recovery = CoordinateRecovery::Pending;
            self.s.status = Status::default();
            self.s.status.state = "Connecting".into();
            self.got_units = false;
            self.auto_reports = false;
            self.queue.push_back("$report_inches".into());
            self.queue.push_back("$Report/Interval=100".into());
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
                if matches!(self.coordinate_recovery, CoordinateRecovery::Writing)
                    || (!self.s.initialized && cmd == "G54")
                {
                    self.coordinate_recovery = CoordinateRecovery::Failed;
                    self.s.phase = "Coordinate restoration failed".into();
                    self.queue.clear();
                    self.s.initialized = false;
                }
                if cmd.starts_with("$LocalFS/Run=") {
                    self.realtime(&[protocol::RESET])?;
                    self.s.job = false;
                }
                if cmd.starts_with("$J=") && self.s.tracing {
                    self.trace = None;
                    self.s.tracing = false;
                    self.s.phase = "Hull trace failed".into();
                }
                if cmd.starts_with("G10") {
                    self.s.status.position_known = false;
                }
            } else if cmd.starts_with("$LocalFS/Run=") {
                self.finish();
            }
            if line == "ok" {
                if cmd == "$Report/Interval=100" {
                    self.auto_reports = true;
                }
                // Refresh promptly after any acknowledged write, rather than waiting
                // for the ordinary heartbeat. Auto reports also force changed WCO/Ov fields.
                self.realtime(b"?")?;
                self.poll = Instant::now();
                if cmd.manual {
                    self.preview_dirty = true;
                    self.queue.push_back("$G".into());
                }
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
            if self
                .pending
                .as_ref()
                .is_some_and(|(cmd, _)| cmd.starts_with("$LocalFS/Run=") || cmd.starts_with("$J="))
            {
                self.pending = None;
            }
            self.trace = None;
            self.s.tracing = false;
            self.queue.retain(|cmd| !cmd.starts_with("$J="));
            self.s.alarm = Some(line.into());
            self.s.status.state = "Alarm".into();
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
            Action::Load { name, text } => {
                ensure!(
                    !self.s.job && !self.s.tracing && self.transfer.is_none(),
                    "Cannot replace a running, paused or uploading job"
                );
                ensure!(
                    !self.s.connected || !self.s.initialized || self.s.status.idle(),
                    "Wait until the controller is idle before opening a file"
                );
                let p = gcode::parse(&text, self.s.options)?;
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
                ensure!(self.stopping.is_none(), "Wait for Stop to finish");
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
                if self.stopping.is_some() {
                    self.reset_controller()?;
                    return Ok(());
                }
                let jogging = self.s.tracing || self.jog.is_some() || self.s.status.state == "Jog";
                self.trace = None;
                self.s.tracing = false;
                self.jog = None;
                self.queue.clear();
                if self.s.status.state.contains("Alarm") {
                    self.reset_controller()?;
                } else {
                    // The last report may still say Hold:0 just after Resume. Always
                    // request a stop and require a newer report before resetting.
                    self.realtime(&[if jogging {
                        protocol::JOG_CANCEL
                    } else {
                        protocol::HOLD
                    }])?;
                    self.pause = if jogging {
                        PauseState::None
                    } else {
                        PauseState::Holding
                    };
                    self.stopping = Some(Instant::now());
                    self.s.phase = "Stopping…".into();
                }
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
                self.realtime(b"?")?;
                self.poll = Instant::now();
            }
            Action::Jog { x, y, speed, step } => {
                ensure!(
                    self.s.can_jog() && !self.s.status.torch,
                    "Jogging requires an idle machine and torch off"
                );
                ensure!(
                    speed.is_finite()
                        && (1.0..=3000.0).contains(&speed)
                        && x.is_finite()
                        && y.is_finite()
                        && (x != 0.0 || y != 0.0)
                        && step.is_finite()
                        && (0.001..=100.0).contains(&step),
                    "Invalid jog speed, direction or increment"
                );
                let length = x.hypot(y);
                let (x, y) = (x / length, y / length);
                if let Some(jog) = &mut self.jog
                    && (jog.x, jog.y, jog.speed, jog.step) == (x, y, speed, step)
                {
                    jog.heartbeat = Instant::now();
                    return Ok(());
                }
                if self.jog.is_some() {
                    self.action(Action::CancelJog)?;
                }
                self.jog = Some(ManualJog {
                    x,
                    y,
                    speed,
                    step,
                    target: Point {
                        x: self.s.status.wpos.x + x * step,
                        y: self.s.status.wpos.y + y * step,
                        z: self.s.status.wpos.z,
                    },
                    heartbeat: Instant::now(),
                    extended: Instant::now(),
                    sent: false,
                    released: false,
                });
                self.preview_dirty = true;
            }
            Action::FinishJog => {
                // A normal release completes the selected increment, including a
                // short tap. Focus loss, Stop and heartbeat expiry cancel instead.
                if let Some(jog) = &mut self.jog {
                    jog.released = true;
                    if jog.sent {
                        self.jog = None;
                    }
                }
            }
            Action::CancelJog => {
                if !self.s.tracing
                    && (self.jog.take().is_some()
                        || self.s.status.state == "Jog"
                        || self.queue.iter().any(|cmd| cmd.starts_with("$J="))
                        || self
                            .pending
                            .as_ref()
                            .is_some_and(|(cmd, _)| cmd.starts_with("$J=")))
                {
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
                        && !self.s.job
                        && !self.s.tracing,
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
                self.queue
                    .push_back(format!("G10L20P1{axis}{value:.6}").into());
                self.queue.push_back("$G".into());
                self.s.status.wco_known = false;
                self.s.status.position_known = false;
                self.preview_dirty = true;
            }
            Action::Connect { address, http } => {
                ensure!(
                    !self.s.job && !self.s.tracing && self.transfer.is_none(),
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
            Action::Console(command) => {
                let command = command.trim();
                ensure!(!command.is_empty(), "Enter a command");
                match command {
                    "?" => {
                        self.s.console("> ?");
                        self.realtime(b"?")?;
                    }
                    "!" => {
                        if self.s.status.state == "Run" {
                            self.s.job = true;
                        }
                        self.action(Action::Pause)?;
                    }
                    "~" => self.action(Action::Resume)?,
                    _ => {
                        ensure!(
                            self.s.connected
                                && self.s.initialized
                                && !self.s.job
                                && !self.s.tracing
                                && !self.s.transferring
                                && (self.s.status.idle() || self.s.status.state.contains("Alarm")),
                            "Ordinary console commands require an idle or alarmed controller; ? ! ~ remain available during jobs"
                        );
                        ensure!(
                            command.is_ascii()
                                && !command.bytes().any(|b| b < 32)
                                && !command.contains(['?', '!', '~']),
                            "Send one command at a time; realtime controls must be sent separately"
                        );
                        self.queue.push_back(QueuedCommand {
                            text: command.into(),
                            manual: true,
                        });
                    }
                }
            }
            Action::TraceHull { speed } => {
                ensure!(
                    self.s.can_jog()
                        && self.s.status.idle()
                        && self.s.status.position_known
                        && !self.s.status.torch
                        && self.stopping.is_none()
                        && self.pending.is_none()
                        && self.queue.is_empty(),
                    "Hull tracing requires an idle machine with the torch off"
                );
                ensure!(
                    speed.is_finite() && (1.0..=3000.0).contains(&speed),
                    "Invalid trace speed"
                );
                let original = &self
                    .s
                    .program
                    .as_ref()
                    .ok_or_else(|| anyhow::anyhow!("Open a job before tracing its hull"))?
                    .original;
                let program = gcode::parse(original, self.s.options)?;
                let mut hull = program.hull();
                ensure!(hull.len() >= 2, "This job has no XY extent to trace");
                let start = self.s.status.wpos;
                let closest = hull
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        a.distance(Point { z: 0.0, ..start })
                            .total_cmp(&b.distance(Point { z: 0.0, ..start }))
                    })
                    .unwrap()
                    .0;
                hull.rotate_left(closest);
                hull.push(hull[0]);
                hull.push(Point { z: 0.0, ..start });
                if program.uploaded != self.s.program.as_ref().unwrap().uploaded {
                    self.need_upload = true;
                    self.failed_transfer = false;
                    self.s.ready = false;
                }
                self.s.program = Some(Arc::new(program));
                self.preview_dirty = true;
                self.s.tracing = true;
                self.s.phase = "Tracing hull…".into();
                self.s.error = None;
                self.s
                    .log(format!("Tracing hull at {speed:.0} mm/min with torch off"));
                self.trace = Some(Trace {
                    points: hull,
                    next: 0,
                    sent: None,
                    speed,
                });
            }
            Action::CancelTrace => {
                self.trace = None;
                self.s.tracing = false;
                self.queue.retain(|cmd| !cmd.starts_with("$J="));
                self.realtime(&[protocol::JOG_CANCEL])?;
                self.s.phase = "Hull trace cancelled".into();
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
                    self.realtime(&[protocol::RESUME])?;
                    self.pause = PauseState::None;
                    self.s.phase = "Running".into();
                } else if sent.elapsed() > Duration::from_secs(5) {
                    self.realtime(&[protocol::RESET])?;
                    self.s.job = false;
                    self.pause = PauseState::None;
                    anyhow::bail!("Torch restore confirmation timed out; job aborted");
                }
            }
            _ => {}
        }
        if matches!(self.pause, PauseState::None) && self.s.job && self.s.status.state == "Run" {
            self.s.phase = "Running".into();
        }
        Ok(())
    }
    fn advance_jog(&mut self) -> Result<()> {
        let Some(jog) = &mut self.jog else {
            return Ok(());
        };
        if jog.heartbeat.elapsed() > Duration::from_millis(250)
            || !self.s.can_jog()
            || self.s.status.torch
        {
            self.action(Action::CancelJog)?;
            return Ok(());
        }
        if self.pending.is_some() || !self.queue.is_empty() {
            return Ok(());
        }
        if jog.sent {
            if jog.extended.elapsed() < Duration::from_millis(100)
                // Keep only a small distance ahead of reported motion. A slow
                // controller must not accumulate a growing queue of increments.
                || jog.target.distance(self.s.status.wpos) > jog.speed / 600.0 + 0.001
            {
                return Ok(());
            }
            let distance = (jog.speed / 600.0 / jog.step).ceil().max(1.0) * jog.step;
            jog.target.x += jog.x * distance;
            jog.target.y += jog.y * distance;
        }
        let command = if jog.sent {
            format!(
                "$J=G21G90X{:.5}Y{:.5}F{:.1}",
                jog.target.x, jog.target.y, jog.speed
            )
        } else {
            // Each tap is relative so consecutive taps still move one increment
            // each when they arrive faster than position feedback.
            format!(
                "$J=G21G91X{:.5}Y{:.5}F{:.1}",
                jog.x * jog.step,
                jog.y * jog.step,
                jog.speed
            )
        };
        self.queue.push_back(command.into());
        jog.sent = true;
        jog.extended = Instant::now();
        if jog.released {
            self.jog = None;
        }
        Ok(())
    }
    fn reset_controller(&mut self) -> Result<()> {
        self.realtime(&[protocol::RESET])?;
        self.s.job = false;
        self.s.tracing = false;
        self.trace = None;
        self.stopping = None;
        self.pause = PauseState::None;
        self.jog = None;
        self.s.ready = false;
        self.queue.clear();
        self.pending = None;
        self.s.initialized = false;
        self.init_started = false;
        self.selected_work_system = false;
        self.coordinate_recovery = CoordinateRecovery::Pending;
        self.got_units = false;
        self.auto_reports = false;
        self.reset_wait = Some(Instant::now());
        self.s.phase = "Stopped".into();
        self.need_verify = self.s.program.is_some();
        self.s.log("Job stopped; controller reset");
        Ok(())
    }
    fn advance_stop(&mut self) -> Result<()> {
        let Some(started) = self.stopping else {
            return Ok(());
        };
        if self.s.status.fresh()
            && self.s.status.at.is_some_and(|at| at > started)
            && (self.s.status.held() || self.s.status.idle())
        {
            self.reset_controller()?;
        } else if started.elapsed() > Duration::from_secs(5) {
            self.reset_controller()?;
            anyhow::bail!(
                "Motion hold timed out; issued an immediate reset. Check the controller alarm before continuing"
            );
        }
        Ok(())
    }
    fn advance_trace(&mut self) -> Result<()> {
        if self.trace.is_none() {
            return Ok(());
        }
        if self.s.status.torch {
            self.reset_controller()?;
            anyhow::bail!("Hull trace stopped because the torch was reported on");
        }
        if self.s.status.state.contains("Alarm") {
            self.trace = None;
            self.s.tracing = false;
            self.s.phase = "Hull trace stopped by alarm".into();
            return Ok(());
        }
        if !self.s.connected
            || !self.s.status.fresh()
            || self.pending.is_some()
            || !self.queue.is_empty()
        {
            return Ok(());
        }
        let trace = self.trace.as_mut().unwrap();
        if let Some(sent) = trace.sent {
            let current = Point {
                z: 0.0,
                ..self.s.status.wpos
            };
            if !self.s.status.idle()
                || self.s.status.at.is_none_or(|at| at <= sent)
                || current.distance(trace.points[trace.next]) > 0.01
            {
                return Ok(());
            }
            trace.next += 1;
            trace.sent = None;
        }
        if trace.next == trace.points.len() {
            self.trace = None;
            self.s.tracing = false;
            self.s.phase = "Hull trace complete".into();
            self.s
                .log("Hull trace complete; returned to starting position");
            return Ok(());
        }
        let point = trace.points[trace.next];
        self.queue.push_back(
            format!("$J=G21G90X{:.3}Y{:.3}F{:.1}", point.x, point.y, trace.speed).into(),
        );
        trace.sent = Some(Instant::now());
        self.s.phase = format!("Tracing hull · {}/{}", trace.next + 1, trace.points.len());
        Ok(())
    }
    fn advance_transfer(&mut self) {
        if self.preview_dirty
            && self.s.initialized
            && self.s.status.idle()
            && self.s.status.position_known
            && !self.s.job
            && !self.s.tracing
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
            && !self.s.tracing
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

    fn connected_worker() -> (crate::simulator::Simulator, Worker) {
        let sim = crate::simulator::Simulator::start().unwrap();
        let (_tx, rx) = mpsc::channel();
        let mut worker = Worker::new(
            sim.address.clone(),
            sim.http.clone(),
            rx,
            Arc::new(Mutex::new(Snapshot::default())),
        );
        worker.link = Some(Link::connect(&sim.address).unwrap());
        worker.s.connected = true;
        worker.s.initialized = true;
        worker.got_units = true;
        worker
            .s
            .status
            .update("<Idle|MPos:10,20,0|WCO:0,0,0|Ov:100,100,100>", false);
        (sim, worker)
    }

    #[test]
    fn offset_report_before_ack_survives_restore_verification() {
        let (_sim, mut worker) = connected_worker();
        let target = Point {
            x: 42.0,
            y: 25.0,
            z: 0.0,
        };
        worker.coordinates.record(&worker.address, target).unwrap();
        worker.coordinate_recovery = CoordinateRecovery::Writing;
        worker.pending = Some(("G10L20P1X42Y25Z0".into(), Instant::now()));
        worker.s.status.wco_known = false;
        worker.s.status.position_known = false;
        worker.line("<Idle|MPos:10,20,0|WCO:-32,-5,0>").unwrap();
        worker.line("ok").unwrap();
        assert!(!worker.recover_coordinates().unwrap());
        worker.line("<Idle|MPos:10,20,0>").unwrap();
        assert!(worker.recover_coordinates().unwrap());
        assert_eq!(worker.s.status.wpos, target);
        assert_eq!(worker.s.status.mpos.x, 10.0);
    }

    #[test]
    fn quick_jog_tap_completes_exactly_one_increment() {
        let (_sim, mut worker) = connected_worker();
        worker
            .action(Action::Jog {
                x: 1.0,
                y: 0.0,
                speed: 600.0,
                step: 0.25,
            })
            .unwrap();
        // Both input messages may arrive before the worker's next motion tick.
        worker.action(Action::FinishJog).unwrap();
        worker.advance_jog().unwrap();
        assert_eq!(worker.queue.len(), 1);
        assert_eq!(worker.queue[0], "$J=G21G91X0.25000Y0.00000F600.0");
        assert!(worker.jog.is_none());
        worker.advance_jog().unwrap();
        assert_eq!(worker.queue.len(), 1);
    }

    #[test]
    fn held_jog_extends_in_increments_without_building_a_backlog() {
        let (_sim, mut worker) = connected_worker();
        worker
            .action(Action::Jog {
                x: 0.0,
                y: -1.0,
                speed: 600.0,
                step: 0.3,
            })
            .unwrap();
        worker.advance_jog().unwrap();
        worker.queue.clear(); // First command acknowledged.
        worker.s.status.wpos.y = 19.7;
        worker.jog.as_mut().unwrap().extended = Instant::now() - Duration::from_millis(101);
        worker.advance_jog().unwrap();
        assert_eq!(worker.queue[0], "$J=G21G90X10.00000Y18.50000F600.0");
        worker.queue.clear();
        // Without position progress, repeated heartbeats must not run farther ahead.
        for _ in 0..5 {
            worker.jog.as_mut().unwrap().extended = Instant::now() - Duration::from_millis(101);
            worker.advance_jog().unwrap();
            assert!(worker.queue.is_empty());
        }
        worker.jog.as_mut().unwrap().heartbeat = Instant::now() - Duration::from_millis(251);
        worker.advance_jog().unwrap();
        assert!(worker.jog.is_none() && worker.queue.is_empty());
        assert!(worker.s.console.back().unwrap().contains("cancel jog"));
    }

    #[test]
    fn cancel_after_releasing_a_tap_removes_unsent_motion() {
        let (_sim, mut worker) = connected_worker();
        worker
            .action(Action::Jog {
                x: 1.0,
                y: 0.0,
                speed: 600.0,
                step: 1.0,
            })
            .unwrap();
        worker.advance_jog().unwrap();
        worker.action(Action::FinishJog).unwrap();
        worker.action(Action::CancelJog).unwrap();
        assert!(worker.jog.is_none() && worker.queue.is_empty());
        // Cancellation must also work after transmission, before either the ack
        // or the first Jog status report has arrived.
        let cancelled = worker.s.console.len();
        worker.pending = Some(("$J=G21G91X1F600".into(), Instant::now()));
        worker.action(Action::CancelJog).unwrap();
        assert_eq!(worker.s.console.len(), cancelled + 1);
    }
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
    #[test]
    fn stop_requires_a_report_newer_than_its_hold_request() {
        let sim = crate::simulator::Simulator::start().unwrap();
        let (_tx, rx) = mpsc::channel();
        let mut worker = Worker::new(
            sim.address.clone(),
            sim.http.clone(),
            rx,
            Arc::new(Mutex::new(Snapshot::default())),
        );
        worker.link = Some(Link::connect(&sim.address).unwrap());
        worker.s.connected = true;
        worker.s.job = true;
        worker.got_units = true;
        worker
            .s
            .status
            .update("<Hold:0|MPos:0,0,0|WCO:0,0,0|Ov:100,100,100>", false);
        worker.action(Action::Stop).unwrap();
        worker.advance_stop().unwrap();
        assert!(
            worker.reset_wait.is_none(),
            "A stale paused report cannot justify reset after Resume"
        );
        worker
            .line("<Hold:0|MPos:0,0,0|WCO:0,0,0|Ov:100,100,100>")
            .unwrap();
        worker.advance_stop().unwrap();
        assert!(worker.reset_wait.is_some());
    }
    #[test]
    fn tracing_preserves_cam_contents_when_initial_torch_speed_changes() {
        let (_tx, rx) = mpsc::channel();
        let mut worker = Worker::new(
            "127.0.0.1:1".into(),
            "http://127.0.0.1:1".into(),
            rx,
            Arc::new(Mutex::new(Snapshot::default())),
        );
        worker.s.connected = true;
        worker.s.initialized = true;
        worker.s.ready = true;
        worker
            .s
            .status
            .update("<Idle|MPos:0,0,0|WCO:0,0,0|Ov:100,100,100>", false);
        let original = "M3\nG1X10F60\nM5";
        let previous = gcode::parse(original, worker.s.options).unwrap();
        assert!(!previous.uploaded.contains("G4"));
        let uploaded = previous.uploaded.clone();
        worker.s.program = Some(Arc::new(previous));
        worker.s.options.initial_speed = 1.0;
        worker.action(Action::TraceHull { speed: 600.0 }).unwrap();
        assert_eq!(worker.s.program.as_ref().unwrap().uploaded, uploaded);
        assert!(worker.s.ready && !worker.need_upload);
    }
}
