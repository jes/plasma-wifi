use crate::{
    coordinates::Coordinates,
    gcode::{self, ParseOptions, Point, Program, TimeParts},
    protocol::{self, Link, Status},
};
use anyhow::{Context, Result, ensure};
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
    ListFiles,
    UploadFile {
        path: PathBuf,
    },
    DownloadFile {
        name: String,
        path: PathBuf,
    },
    TraceHull {
        speed: f64,
    },
    JogSpeed {
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
    pub files: Vec<protocol::RemoteFile>,
    pub files_message: String,
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
    pub acceleration: [Option<f64>; 2],
    pub planner_blocks: Option<usize>,
    pub modal: String,
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
            files: Vec::new(),
            files_message: String::new(),
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
            acceleration: [None; 2],
            planner_blocks: None,
            modal: String::new(),
        }
    }
}
impl Snapshot {
    pub fn can_manage_files(&self) -> bool {
        self.connected
            && self.status.fresh()
            && (self.status.idle() || self.status.state == "Alarm")
            && !self.status.torch
            && !self.job
            && !self.tracing
            && !self.transferring
    }
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
            && self.acceleration.iter().all(Option::is_some)
            && self.planner_blocks.is_some()
            && self.status.fresh()
            && self.status.position_known
            && self.status.wco_known
            && !self.job
            && !self.transferring
            && !self.tracing
            && (self.status.idle() || self.status.state == "Jog")
    }
    fn jog_lookahead(
        &self,
        direction: Point,
        speed: f64,
        previous: f64,
        ack_delay: f64,
    ) -> Option<JogLookahead> {
        let length = direction.x.hypot(direction.y);
        if length < 1e-12 {
            return None;
        }
        let mut acceleration = f64::INFINITY;
        let mut velocity = speed.max(previous).max(self.status.feed) / 60.0;
        for (component, acceleration_limit, rate) in [
            (
                direction.x.abs() / length,
                self.acceleration[0]?,
                self.options.rapid.x,
            ),
            (
                direction.y.abs() / length,
                self.acceleration[1]?,
                self.options.rapid.y,
            ),
        ] {
            if component > 1e-12 {
                acceleration = acceleration.min(acceleration_limit / component);
                velocity = velocity.min(rate / 60.0 / component);
            }
        }
        let report_age = self.status.at.map_or(0.0, |at| at.elapsed().as_secs_f64());
        // Keep the stopping distance available even while a report and a pair
        // of command round trips are in flight. Acks include planner backpressure.
        let refill = velocity * velocity / (2.0 * acceleration)
            + velocity * (0.15 + report_age + 2.0 * ack_delay.max(0.01));
        // Reserve slots for the active block, the next packet and an ack in
        // flight. Slow acceleration must enlarge packets, not fill all slots
        // with too little physical distance to reach the requested velocity.
        // An increment boundary can add a shorter block between full packets.
        let packets = self.planner_blocks?.saturating_sub(4).max(2) as f64 / 2.0;
        let segment = (velocity * 0.1)
            .max(velocity * (2.0 * ack_delay + 0.02))
            .max(refill / packets)
            .max(0.001);
        (refill.is_finite() && segment.is_finite()).then_some(JogLookahead { refill, segment })
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
    active_speed: f64,
    cursor: Point,
}

struct JogLookahead {
    refill: f64,
    segment: f64,
}

struct ManualJog {
    x: f64,
    y: f64,
    speed: f64,
    active_speed: f64,
    cursor: Point,
    step: f64,
    target: Point,
    origin: Point,
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
    FilesMessage(String),
    FilesDone(Result<FilesResult>),
}
enum FilesResult {
    Listed(Vec<protocol::RemoteFile>),
    Uploaded {
        name: String,
        info: protocol::FileInfo,
    },
    Downloaded {
        name: String,
        path: PathBuf,
        bytes: usize,
    },
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
    jog_ack_delay: f64,
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
            jog_ack_delay: 0.01,
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
                        self.s.modal.clear();
                        self.s.initialized = false;
                        self.s.acceleration = [None; 2];
                        self.s.planner_blocks = None;
                        self.jog_ack_delay = 0.01;
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
        self.s.files.clear();
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
        let establishing_position =
            !self.s.initialized || !self.s.status.position_known || !self.s.status.wco_known;
        if self.poll.elapsed()
            > Duration::from_millis(if self.auto_reports && !establishing_position {
                1000
            } else {
                100
            })
        {
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
            self.s.acceleration = [None; 2];
            self.s.planner_blocks = None;
            self.jog_ack_delay = 0.01;
            self.selected_work_system = false;
            self.coordinate_recovery = CoordinateRecovery::Pending;
            self.s.status = Status::default();
            self.s.status.state = "Connecting".into();
            self.s.modal.clear();
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
            "$/axes/x/acceleration_mm_per_sec2=",
            "$/axes/y/acceleration_mm_per_sec2=",
        ]
        .into_iter()
        .enumerate()
        {
            if let Some(value) = line
                .strip_prefix(key)
                .and_then(|v| v.parse::<f64>().ok())
                .filter(|v| v.is_finite() && *v > 0.0)
            {
                self.s.acceleration[axis] = Some(value);
            }
        }
        if let Some(blocks) = line
            .strip_prefix("$/planner_blocks=")
            .and_then(|v| v.parse::<usize>().ok())
            .filter(|v| (10..=120).contains(v))
        {
            self.s.planner_blocks = Some(blocks);
        }
        for (axis, key) in [
            ('x', "$/axes/x/max_rate_mm_per_min="),
            ('y', "$/axes/y/max_rate_mm_per_min="),
            ('z', "$/axes/z/max_rate_mm_per_min="),
        ] {
            if let Some(v) = line
                .strip_prefix(key)
                .and_then(|v| v.parse::<f64>().ok())
                .filter(|v| v.is_finite() && *v > 0.0)
            {
                match axis {
                    'x' => self.s.options.rapid.x = v,
                    'y' => self.s.options.rapid.y = v,
                    _ => self.s.options.rapid.z = v,
                }
            }
        }
        if let Some(modal) = line.strip_prefix("[GC:").and_then(|s| s.strip_suffix(']')) {
            self.s.modal = modal.into();
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
            && let Some((cmd, sent)) = self.pending.take()
        {
            if cmd.starts_with("$J=") && line == "ok" {
                self.jog_ack_delay = (self.jog_ack_delay * 0.9).max(sent.elapsed().as_secs_f64());
            }
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
                    "$axes/x/acceleration_mm_per_sec2",
                    "$axes/y/acceleration_mm_per_sec2",
                    "$planner_blocks",
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
            action @ (Action::ListFiles
            | Action::UploadFile { .. }
            | Action::DownloadFile { .. }) => {
                ensure!(
                    self.s.can_manage_files(),
                    "File transfers require an idle controller or Alarm, with the torch off"
                );
                ensure!(
                    self.jog.is_none()
                        && self.pending.is_none()
                        && self.queue.is_empty()
                        && self.stopping.is_none()
                        && self.reset_wait.is_none(),
                    "Wait for the current controller command to finish"
                );
                if let Action::UploadFile { path } = &action {
                    let name = path
                        .file_name()
                        .and_then(|name| name.to_str())
                        .context("Invalid local filename")?;
                    protocol::validate_filename(name)?;
                    if name == "plasma.nc" {
                        // An arbitrary replacement is not the verified, processed job.
                        self.s.ready = false;
                        self.need_upload = false;
                        self.need_verify = false;
                        self.failed_transfer = true;
                        self.s.upload =
                            "plasma.nc replaced through Files; reopen G-code before Run".into();
                    }
                }
                if let Action::DownloadFile { name, .. } = &action {
                    protocol::validate_filename(name)?;
                }
                let base = self.http.clone();
                let (tx, rx) = mpsc::channel();
                self.transfer = Some(rx);
                self.s.transferring = true;
                if self.s.error.as_deref() == Some(self.s.files_message.as_str()) {
                    self.s.error = None;
                }
                self.s.files_message = "Transferring controller files…".into();
                thread::spawn(move || {
                    let result = (|| -> Result<FilesResult> {
                        match action {
                            Action::ListFiles => {
                                Ok(FilesResult::Listed(protocol::list_files(&base)?))
                            }
                            Action::UploadFile { path } => {
                                let name = path
                                    .file_name()
                                    .and_then(|name| name.to_str())
                                    .unwrap()
                                    .to_owned();
                                let data = std::fs::read(&path)
                                    .with_context(|| format!("Read {}", path.display()))?;
                                let info = protocol::upload_file(&base, &name, &data, |message| {
                                    let _ = tx.send(Transfer::FilesMessage(message.into()));
                                })?;
                                Ok(FilesResult::Uploaded { name, info })
                            }
                            Action::DownloadFile { name, path } => {
                                let bytes = protocol::download_file(&base, &name, &path)?;
                                Ok(FilesResult::Downloaded { name, path, bytes })
                            }
                            _ => unreachable!(),
                        }
                    })();
                    let _ = tx.send(Transfer::FilesDone(result));
                });
            }
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
                    && (jog.x, jog.y, jog.step) == (x, y, step)
                {
                    jog.speed = speed;
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
                    active_speed: speed,
                    cursor: self.s.status.wpos,
                    step,
                    target: Point {
                        x: self.s.status.wpos.x + x * step,
                        y: self.s.status.wpos.y + y * step,
                        z: self.s.status.wpos.z,
                    },
                    origin: self.s.status.wpos,
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
                    // Finish only the increment boundary needed by motion
                    // already sent, rather than an unsent lookahead target.
                    let sent_distance = (jog.cursor.x - jog.origin.x) * jog.x
                        + (jog.cursor.y - jog.origin.y) * jog.y;
                    let distance = ((sent_distance / jog.step - 1e-9).ceil().max(1.0)) * jog.step;
                    jog.target = Point {
                        x: jog.origin.x + jog.x * distance,
                        y: jog.origin.y + jog.y * distance,
                        z: jog.origin.z,
                    };
                    jog.released = true;
                    if jog.sent && jog.cursor.distance(jog.target) < 1e-9 {
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
            Action::JogSpeed { speed } => {
                ensure!(
                    speed.is_finite() && (1.0..=3000.0).contains(&speed),
                    "Invalid trace speed"
                );
                if let Some(trace) = &mut self.trace {
                    trace.speed = speed;
                }
                if let Some(jog) = &mut self.jog {
                    jog.speed = speed;
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
                    active_speed: speed,
                    cursor: Point { z: 0.0, ..start },
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
        if !jog.released && jog.heartbeat.elapsed() > Duration::from_millis(250)
            || !self.s.can_jog()
            || self.s.status.torch
        {
            self.action(Action::CancelJog)?;
            return Ok(());
        }
        if self.pending.is_some() || !self.queue.is_empty() {
            return Ok(());
        }
        let lookahead = self
            .s
            .jog_lookahead(
                Point {
                    x: jog.x,
                    y: jog.y,
                    z: 0.0,
                },
                jog.speed,
                jog.active_speed,
                self.jog_ack_delay,
            )
            .context("Controller acceleration or planner capacity unavailable for jogging")?;
        if !jog.released
            && jog.extended.elapsed() >= Duration::from_millis(100)
            && jog.target.distance(self.s.status.wpos) <= lookahead.refill + 0.001
        {
            let needed = (lookahead.refill + lookahead.segment
                - jog.target.distance(self.s.status.wpos))
            .max(lookahead.segment);
            let distance = (needed / jog.step).ceil().max(1.0) * jog.step;
            jog.target.x += jog.x * distance;
            jog.target.y += jog.y * distance;
            jog.extended = Instant::now();
        }
        let remaining = jog.cursor.distance(jog.target);
        if remaining < 1e-9 {
            if jog.released {
                self.jog = None;
            }
            return Ok(());
        }
        // Keep enough physical distance ahead for braking and communication,
        // without using up the planner's capacity with packets that are too short.
        if jog.sent && jog.cursor.distance(self.s.status.wpos) > lookahead.refill + 0.001 {
            return Ok(());
        }
        let point = jog
            .cursor
            .lerp(jog.target, (lookahead.segment / remaining).min(1.0));
        let command = if jog.sent {
            format!("$J=G21G90X{:.5}Y{:.5}F{:.1}", point.x, point.y, jog.speed)
        } else {
            // Relative first segments preserve separate taps received before
            // position feedback; subsequent segments use absolute targets.
            format!(
                "$J=G21G91X{:.5}Y{:.5}F{:.1}",
                point.x - jog.cursor.x,
                point.y - jog.cursor.y,
                jog.speed
            )
        };
        self.queue.push_back(command.into());
        jog.cursor = point;
        jog.sent = true;
        jog.active_speed = jog.speed;
        if jog.released && jog.cursor.distance(jog.target) < 1e-9 {
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
        let current = Point {
            z: 0.0,
            ..self.s.status.wpos
        };
        if let Some(sent) = trace.sent
            && trace.cursor.distance(trace.points[trace.next]) < 1e-9
        {
            if !self.s.status.idle()
                || self.s.status.at.is_none_or(|at| at <= sent)
                || current.distance(trace.points[trace.next]) > 0.01
            {
                return Ok(());
            }
            trace.next += 1;
            trace.sent = None;
            trace.active_speed = trace.speed;
        }
        while trace.sent.is_none()
            && trace.next < trace.points.len()
            && trace.cursor.distance(trace.points[trace.next]) < 1e-9
        {
            trace.next += 1;
        }
        if trace.next == trace.points.len() {
            self.trace = None;
            self.s.tracing = false;
            self.s.phase = "Hull trace complete".into();
            self.s
                .log("Hull trace complete; returned to starting position");
            return Ok(());
        }
        let target = trace.points[trace.next];
        let remaining = trace.cursor.distance(target);
        let lookahead = self
            .s
            .jog_lookahead(
                Point {
                    x: target.x - trace.cursor.x,
                    y: target.y - trace.cursor.y,
                    z: 0.0,
                },
                trace.speed,
                trace.active_speed,
                self.jog_ack_delay,
            )
            .context("Controller acceleration or planner capacity unavailable for hull tracing")?;
        if trace.sent.is_some() && trace.cursor.distance(current) > lookahead.refill + 0.001 {
            return Ok(());
        }
        let point = trace
            .cursor
            .lerp(target, (lookahead.segment / remaining).min(1.0));
        self.queue.push_back(
            format!("$J=G21G90X{:.3}Y{:.3}F{:.1}", point.x, point.y, trace.speed).into(),
        );
        trace.cursor = point;
        trace.sent = Some(Instant::now());
        trace.active_speed = trace.speed;
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
                    Transfer::FilesMessage(message) => self.s.files_message = message,
                    Transfer::FilesDone(result) => {
                        self.transfer = None;
                        self.s.transferring = false;
                        match result {
                            Ok(FilesResult::Listed(files)) => {
                                self.s.files_message =
                                    format!("{} files on controller", files.len());
                                self.s.files = files;
                            }
                            Ok(FilesResult::Uploaded { name, info }) => {
                                self.s.files.retain(|file| file.name != name);
                                self.s.files.push(protocol::RemoteFile {
                                    name: name.clone(),
                                    bytes: info.bytes,
                                });
                                self.s.files.sort_by(|a, b| a.name.cmp(&b.name));
                                self.s.free = info.free;
                                self.s.files_message = format!(
                                    "Uploaded {name} · {} bytes · contents verified",
                                    info.bytes
                                );
                                self.s.log(self.s.files_message.clone());
                            }
                            Ok(FilesResult::Downloaded { name, path, bytes }) => {
                                self.s.files_message = format!(
                                    "Downloaded {name} · {bytes} bytes → {}",
                                    path.display()
                                );
                                self.s.log(self.s.files_message.clone());
                            }
                            Err(error) => {
                                self.s.files_message = format!("File transfer failed: {error:#}");
                                self.fail(self.s.files_message.clone());
                            }
                        }
                    }
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
        worker.s.acceleration = [Some(100.0); 2];
        worker.s.planner_blocks = Some(16);
        worker.got_units = true;
        worker
            .s
            .status
            .update("<Idle|MPos:10,20,0|WCO:0,0,0|Ov:100,100,100>", false);
        (sim, worker)
    }

    // A separate, deterministic 1D planner model: it accelerates at the configured
    // rate, assumes a stop at the last received endpoint, has finite planner slots,
    // and delays both command acknowledgements and position reports. Unlike the
    // loopback simulator, it does not teleport to commanded targets.
    fn check_sustained_speed(trace: bool, acceleration: f64, blocks: usize, one_way: f64) {
        let (_sim, mut worker) = connected_worker();
        worker
            .s
            .status
            .update("<Idle|MPos:0,0,0|WCO:0,0,0|Ov:100,100,100>", false);
        worker.s.acceleration = [Some(acceleration); 2];
        worker.s.planner_blocks = Some(blocks);
        if trace {
            worker.s.program = Some(Arc::new(
                gcode::parse("G21G90\nG0X10000Y0\nG1Y100F3000\nX0\nY0", worker.s.options).unwrap(),
            ));
            worker.action(Action::TraceHull { speed: 3000.0 }).unwrap();
        } else {
            worker
                .action(Action::Jog {
                    x: 1.0,
                    y: 0.0,
                    speed: 3000.0,
                    step: 0.1,
                })
                .unwrap();
        }
        struct InFlight {
            command: QueuedCommand,
            endpoint: f64,
            sent: f64,
            arrives: f64,
            acknowledged: Option<f64>,
        }
        let mut in_flight: Option<InFlight> = None;
        let mut planner: VecDeque<f64> = VecDeque::new();
        let mut reports: VecDeque<(f64, f64, f64, bool)> = VecDeque::new();
        let (mut position, mut velocity) = (0.0_f64, 0.0_f64);
        let (mut next_report, mut last_report, mut extension_at) = (0.0, 0.0, 0.0);
        let duration = 50.0 / acceleration + 5.0;
        let dt = 0.002;
        let mut sustained = 50.0_f64;
        let mut max_blocks = 0;
        for tick in 0..(duration / dt).ceil() as usize {
            let now = tick as f64 * dt;
            if let Some(&endpoint) = planner.back() {
                let braking_speed = (2.0 * acceleration * (endpoint - position).max(0.0)).sqrt();
                let desired = 50.0_f64.min(braking_speed);
                let next_velocity = if desired > velocity {
                    (velocity + acceleration * dt).min(desired)
                } else {
                    (velocity - acceleration * dt).max(desired)
                };
                position = (position + (velocity + next_velocity) * 0.5 * dt).min(endpoint);
                velocity = if position >= endpoint {
                    0.0
                } else {
                    next_velocity
                };
                while planner
                    .front()
                    .is_some_and(|&point| point <= position + 1e-9)
                {
                    planner.pop_front();
                }
            } else {
                velocity = 0.0;
            }
            if let Some(packet) = &mut in_flight
                && now >= packet.arrives
                && packet.acknowledged.is_none()
                && planner.len() < blocks - 1
            {
                planner.push_back(packet.endpoint);
                packet.acknowledged = Some(now + one_way);
                max_blocks = max_blocks.max(planner.len());
            }
            if in_flight
                .as_ref()
                .is_some_and(|packet| packet.acknowledged.is_some_and(|ack| now >= ack))
            {
                let packet = in_flight.take().unwrap();
                worker.pending = Some((
                    packet.command,
                    Instant::now() - Duration::from_secs_f64(now - packet.sent),
                ));
                worker.line("ok").unwrap();
            }
            if now >= next_report {
                reports.push_back((now + one_way, position, velocity, !planner.is_empty()));
                next_report = now + 0.1;
            }
            while reports.front().is_some_and(|sample| now >= sample.0) {
                let (_, position, velocity, moving) = reports.pop_front().unwrap();
                worker.s.status.wpos.x = position;
                worker.s.status.feed = velocity * 60.0;
                worker.s.status.state = if moving { "Jog" } else { "Idle" }.into();
                last_report = now;
            }
            worker.s.status.at = Some(Instant::now() - Duration::from_secs_f64(now - last_report));
            let before = worker.jog.as_ref().map(|jog| jog.target);
            if let Some(jog) = &mut worker.jog {
                jog.heartbeat = Instant::now();
                jog.extended = Instant::now() - Duration::from_secs_f64(now - extension_at);
            }
            if trace {
                worker.advance_trace().unwrap();
            } else {
                worker.advance_jog().unwrap();
            }
            if before != worker.jog.as_ref().map(|jog| jog.target) {
                extension_at = now;
            }
            if in_flight.is_none()
                && let Some(command) = worker.queue.pop_front()
            {
                let endpoint = if trace {
                    worker.trace.as_ref().unwrap().cursor.x
                } else {
                    worker.jog.as_ref().unwrap().cursor.x
                };
                worker.pending = Some((command.clone(), Instant::now()));
                in_flight = Some(InFlight {
                    command,
                    endpoint,
                    sent: now,
                    arrives: now + one_way,
                    acknowledged: None,
                });
            }
            if now > duration - 2.0 {
                sustained = sustained.min(velocity);
            }
        }
        assert!(
            sustained >= 49.5,
            "trace={trace}, a={acceleration}, slots={blocks}, RTT={}: sustained {sustained} mm/s",
            one_way * 2.0
        );
        assert!(
            max_blocks < blocks - 1,
            "trace={trace}, a={acceleration}, slots={blocks}, one-way={one_way}: {max_blocks} planner blocks used, sustained {sustained}"
        );
    }

    #[test]
    fn jog_and_trace_reach_full_speed_with_acceleration_and_delayed_feedback() {
        for trace in [false, true] {
            for (acceleration, slots, one_way) in
                [(100.0, 16, 0.03), (100.0, 10, 0.07), (5.0, 10, 0.07)]
            {
                check_sustained_speed(trace, acceleration, slots, one_way);
            }
        }
    }

    #[test]
    fn lookahead_uses_the_limiting_axis_and_waits_for_actual_controller_settings() {
        let (_sim, mut worker) = connected_worker();
        worker.s.acceleration = [Some(100.0), Some(25.0)];
        worker.s.options.rapid.y = 600.0;
        worker.s.status.at = Some(Instant::now());
        // Direction (.6,.8): Y limits speed to 12.5 mm/s and path
        // acceleration to 31.25 mm/s². Braking needs 2.5 mm, plus margin.
        let lookahead = worker
            .s
            .jog_lookahead(
                Point {
                    x: 0.6,
                    y: 0.8,
                    z: 0.0,
                },
                3000.0,
                3000.0,
                0.01,
            )
            .unwrap();
        assert!((4.625..4.65).contains(&lookahead.refill));
        assert!((lookahead.segment - 1.25).abs() < 1e-9);
        worker.s.acceleration[0] = None;
        assert!(!worker.s.can_jog());
        worker
            .line("$/axes/x/acceleration_mm_per_sec2=nan")
            .unwrap();
        assert!(!worker.s.can_jog());
        worker.line("$/axes/x/acceleration_mm_per_sec2=80").unwrap();
        assert!(worker.s.can_jog());
        worker.line("Grbl 4.1 [FluidNC reboot]").unwrap();
        assert_eq!(worker.s.acceleration, [None; 2]);
        assert_eq!(worker.s.planner_blocks, None);
        assert!(!worker.s.can_jog());
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
        assert_eq!(worker.queue[0], "$J=G21G90X10.00000Y18.70000F600.0");
        worker.queue.clear();
        // Without position progress, repeated heartbeats must not run farther ahead.
        for _ in 0..5 {
            worker.jog.as_mut().unwrap().extended = Instant::now() - Duration::from_millis(101);
            worker.advance_jog().unwrap();
            let jog = worker.jog.as_ref().unwrap();
            let increments = (20.0 - jog.target.y) / 0.3;
            assert!((increments - increments.round()).abs() < 1e-8);
            assert!(jog.cursor.distance(worker.s.status.wpos) < 3.5);
            worker.queue.clear();
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
    fn controller_file_transfers_block_motion_and_overlapping_operations() {
        let (_sim, mut worker) = connected_worker();
        worker.s.ready = true;
        worker.action(Action::ListFiles).unwrap();
        assert!(worker.s.transferring);
        assert!(!worker.s.can_run() && !worker.s.can_jog());
        assert!(worker.action(Action::Run).is_err());
        assert!(
            worker
                .action(Action::Jog {
                    x: 1.0,
                    y: 0.0,
                    speed: 600.0,
                    step: 1.0
                })
                .is_err()
        );
        assert!(worker.action(Action::Console("G0X100".into())).is_err());
        assert!(worker.action(Action::ListFiles).is_err());
        assert!(
            worker
                .action(Action::Load {
                    name: "other.nc".into(),
                    text: "G0X1".into()
                })
                .is_err()
        );
        assert!(
            worker
                .action(Action::Connect {
                    address: "127.0.0.1:1".into(),
                    http: "http://127.0.0.1:1".into()
                })
                .is_err()
        );
    }

    #[test]
    fn controller_files_reject_active_motion_and_allow_uninitialized_alarm() {
        let (_sim, mut worker) = connected_worker();
        for state in ["Run", "Jog", "Hold:0", "Hold:1"] {
            worker.s.status.state = state.into();
            assert!(worker.action(Action::ListFiles).is_err());
        }
        worker.s.status.state = "Idle".into();
        worker.s.status.torch = true;
        assert!(worker.action(Action::ListFiles).is_err());
        worker.s.status.torch = false;
        worker.s.status.state = "Alarm".into();
        worker.s.initialized = false;
        worker.action(Action::ListFiles).unwrap();
        assert!(worker.s.transferring);
    }

    #[test]
    fn changing_jog_speed_queues_new_feeds_without_cancelling_motion() {
        let (_sim, mut worker) = connected_worker();
        worker
            .action(Action::Jog {
                x: 1.0,
                y: 0.0,
                speed: 600.0,
                step: 100.0,
            })
            .unwrap();
        worker.advance_jog().unwrap();
        worker.queue.clear();
        let target = worker.jog.as_ref().unwrap().target;
        for speed in [2400.0, 600.0] {
            worker.s.status.wpos = worker.jog.as_ref().unwrap().cursor;
            worker.s.status.state = "Jog".into();
            worker
                .action(Action::Jog {
                    x: 1.0,
                    y: 0.0,
                    speed,
                    step: 100.0,
                })
                .unwrap();
            worker.advance_jog().unwrap();
            assert!(
                worker
                    .queue
                    .pop_front()
                    .unwrap()
                    .ends_with(format!("F{speed:.1}").as_str())
            );
            assert_eq!(worker.jog.as_ref().unwrap().target, target);
            assert!(
                !worker
                    .s
                    .console
                    .iter()
                    .any(|line| line.contains("cancel jog"))
            );
        }
        worker.action(Action::FinishJog).unwrap();
        assert!(worker.jog.as_ref().unwrap().released);
    }

    #[test]
    fn changing_trace_speed_keeps_the_edge_and_does_not_cancel_motion() {
        let (_sim, mut worker) = connected_worker();
        worker.s.program = Some(Arc::new(
            gcode::parse(
                "G21G90\nG0X0Y0\nG1X100Y0F600\nY100\nX0\nY0",
                worker.s.options,
            )
            .unwrap(),
        ));
        worker.action(Action::TraceHull { speed: 600.0 }).unwrap();
        worker.advance_trace().unwrap();
        worker.queue.clear();
        let vertex = worker.trace.as_ref().unwrap().points[0];
        for speed in [2400.0, 600.0] {
            worker.s.status.wpos = worker.trace.as_ref().unwrap().cursor;
            worker.s.status.state = "Jog".into();
            worker.action(Action::JogSpeed { speed }).unwrap();
            worker.advance_trace().unwrap();
            assert!(
                worker
                    .queue
                    .pop_front()
                    .unwrap()
                    .ends_with(format!("F{speed:.1}").as_str())
            );
            let trace = worker.trace.as_ref().unwrap();
            assert_eq!(trace.next, 0);
            assert_eq!(trace.points[0], vertex);
            assert!(
                !worker
                    .s
                    .console
                    .iter()
                    .any(|line| line.contains("cancel jog"))
            );
        }
        // Already buffered fast motion is retained, but cannot grow while
        // position feedback is unchanged after reducing the feed.
        let before = worker.trace.as_ref().unwrap().cursor;
        for _ in 0..10 {
            worker.advance_trace().unwrap();
            worker.queue.clear();
        }
        assert_eq!(worker.trace.as_ref().unwrap().cursor, before);
        worker.action(Action::CancelTrace).unwrap();
        worker.advance_trace().unwrap();
        assert!(worker.trace.is_none() && worker.queue.is_empty() && !worker.s.tracing);
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
        worker.s.acceleration = [Some(100.0); 2];
        worker.s.planner_blocks = Some(16);
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
