//! Optional Linux diagnostics. No monitor operation runs on the controller/UI thread.
use std::{
    collections::VecDeque,
    io::Read,
    net::{IpAddr, SocketAddr},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};

const WINDOW: Duration = Duration::from_secs(10);
const COMMAND_TIMEOUT: Duration = Duration::from_millis(800);
const OUTPUT_LIMIT: usize = 64 * 1024;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Checking,
    Wireless,
    Other,
    Unavailable,
    Demo,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EventKind {
    Disconnect,
    Deauthentication,
    Reconnect,
    Authentication,
    BeaconLoss,
}
#[derive(Clone, Debug)]
pub struct Event {
    pub at: Instant,
    pub kind: EventKind,
    pub text: String,
}
#[derive(Clone, Debug)]
pub struct Snapshot {
    controller_address: String,
    pub kind: Kind,
    pub interface: Option<String>,
    pub destination: Option<IpAddr>,
    pub route_current: bool,
    pub associated: Option<bool>,
    pub ssid: Option<String>,
    pub bssid: Option<String>,
    pub signal: Option<f64>,
    pub tx_bitrate: Option<f64>,
    pub rx_bitrate: Option<f64>,
    pub retries_per_second: Option<f64>,
    pub failures_per_second: Option<f64>,
    pub beacon_losses: Option<u64>,
    pub window_seconds: Option<f64>,
    pub event_sources: usize,
    pub events: VecDeque<Event>,
    pub sampled_at: Option<Instant>,
    pub notes: Vec<String>,
}
impl Default for Snapshot {
    fn default() -> Self {
        Self {
            controller_address: String::new(),
            kind: Kind::Checking,
            interface: None,
            destination: None,
            route_current: false,
            associated: None,
            ssid: None,
            bssid: None,
            signal: None,
            tx_bitrate: None,
            rx_bitrate: None,
            retries_per_second: None,
            failures_per_second: None,
            beacon_losses: None,
            window_seconds: None,
            event_sources: 0,
            events: VecDeque::new(),
            sampled_at: None,
            notes: vec![],
        }
    }
}
impl Snapshot {
    pub fn event_count(&self, kind: EventKind) -> Option<usize> {
        (self.event_sources > 0
            || self.kind == Kind::Demo
            || self.sampled_at.is_some()
                && matches!(kind, EventKind::Disconnect | EventKind::Reconnect))
        .then(|| {
            self.events
                .iter()
                .filter(|event| event.kind == kind && event.at.elapsed() < Duration::from_secs(60))
                .count()
        })
    }
    fn clear_radio(&mut self) {
        self.associated = None;
        self.ssid = None;
        self.bssid = None;
        self.signal = None;
        self.tx_bitrate = None;
        self.rx_bitrate = None;
        self.retries_per_second = None;
        self.failures_per_second = None;
        self.beacon_losses = None;
        self.window_seconds = None;
        self.sampled_at = None;
    }
    fn event(&mut self, kind: EventKind, text: String) {
        if kind == EventKind::Deauthentication {
            self.event(EventKind::Disconnect, text.clone());
        }
        // Merge reports from different observers, while preserving real bursts
        // from the same observer and disconnects separated by a reconnect.
        let source = text.split(':').next();
        for event in self.events.iter().rev() {
            if matches!(
                (kind, event.kind),
                (
                    EventKind::Disconnect | EventKind::Deauthentication,
                    EventKind::Reconnect
                ) | (EventKind::Reconnect, EventKind::Disconnect)
            ) {
                break;
            }
            if event.kind == kind {
                if event.at.elapsed() < Duration::from_secs(2)
                    && event.text.split(':').next() != source
                {
                    return;
                }
                break;
            }
        }
        self.events.push_back(Event {
            at: Instant::now(),
            kind,
            text,
        });
        while self.events.len() > 64 {
            self.events.pop_front();
        }
    }
}

#[derive(Clone, PartialEq)]
struct Target {
    address: String,
    peer: Option<IpAddr>,
}
pub struct Monitor {
    shared: Arc<Mutex<Snapshot>>,
    target: Arc<Mutex<Target>>,
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}
impl Monitor {
    pub fn start(address: String, demo: bool) -> Self {
        let snapshot = if demo {
            Snapshot {
                kind: Kind::Demo,
                controller_address: address.clone(),
                associated: Some(true),
                signal: Some(-58.0),
                tx_bitrate: Some(72.2),
                rx_bitrate: Some(65.0),
                retries_per_second: Some(0.2),
                failures_per_second: Some(0.0),
                beacon_losses: Some(0),
                window_seconds: Some(10.0),
                notes: vec![
                    "Simulated readings; demo does not run any network diagnostics.".into(),
                ],
                ..Default::default()
            }
        } else {
            Snapshot::default()
        };
        let mut monitor = Self {
            shared: Arc::new(Mutex::new(snapshot)),
            target: Arc::new(Mutex::new(Target {
                address,
                peer: None,
            })),
            stop: Arc::new(AtomicBool::new(false)),
            handle: None,
        };
        if !demo {
            let (shared, target, stop) = (
                monitor.shared.clone(),
                monitor.target.clone(),
                monitor.stop.clone(),
            );
            monitor.handle = thread::Builder::new()
                .name("wifi-diagnostics".into())
                .spawn(move || {
                    Worker::new(Tools::discover(), stop.clone()).run(shared, target);
                })
                .ok();
            if monitor.handle.is_none() {
                let mut snapshot = monitor.shared.lock().unwrap();
                snapshot.kind = Kind::Unavailable;
                snapshot
                    .notes
                    .push("Could not start diagnostics worker".into());
            }
        }
        monitor
    }
    pub fn snapshot(&self, address: &str, peer: Option<IpAddr>) -> Snapshot {
        let mut target = self.target.lock().unwrap();
        if target.address != address {
            *target = Target {
                address: address.into(),
                peer,
            };
            // Do not show the previous controller's readings while retargeting.
            return Snapshot::default();
        }
        if peer.is_some() {
            target.peer = peer;
        }
        let snapshot = self.shared.lock().unwrap().clone();
        if snapshot.controller_address != address {
            Snapshot::default()
        } else {
            snapshot
        }
    }
}
impl Drop for Monitor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

#[derive(Clone)]
struct Tools {
    paths: Vec<(&'static str, Option<PathBuf>)>,
}
impl Tools {
    fn discover() -> Self {
        let search = std::env::var_os("PATH").unwrap_or_default();
        let mut directories: Vec<_> = std::env::split_paths(&search).collect();
        directories.extend([PathBuf::from("/usr/sbin"), PathBuf::from("/sbin")]);
        Self {
            paths: ["ip", "iw", "journalctl", "stdbuf", "getent"]
                .into_iter()
                .map(|name| {
                    (
                        name,
                        directories
                            .iter()
                            .map(|dir| dir.join(name))
                            .find(|path| path.is_file()),
                    )
                })
                .collect(),
        }
    }
    fn path(&self, name: &str) -> Result<&Path, String> {
        self.paths
            .iter()
            .find(|(tool, _)| *tool == name)
            .and_then(|(_, path)| path.as_deref())
            .ok_or_else(|| format!("{name} is not installed"))
    }
    fn command(&self, name: &str, args: &[&str]) -> Result<Command, String> {
        let mut command = Command::new(self.path(name)?);
        command
            .args(args)
            .env("LC_ALL", "C")
            .stdin(Stdio::null())
            .stderr(Stdio::null());
        Ok(command)
    }
    fn read(&self, name: &str, args: &[&str], stop: &AtomicBool) -> Result<String, String> {
        read_command(self.command(name, args)?, stop, COMMAND_TIMEOUT)
            .map_err(|error| format!("{name}: {error}"))
    }
}
fn read_command(
    mut command: Command,
    stop: &AtomicBool,
    timeout: Duration,
) -> Result<String, String> {
    let mut child = command
        .stdout(Stdio::piped())
        .spawn()
        .map_err(|e| e.to_string())?;
    let output = child.stdout.take().unwrap();
    let (tx, rx) = mpsc::sync_channel(1);
    thread::spawn(move || {
        let mut bytes = Vec::new();
        let result = output
            .take((OUTPUT_LIMIT + 1) as u64)
            .read_to_end(&mut bytes);
        let _ = tx.send((result, bytes));
    });
    let started = Instant::now();
    let result = loop {
        if stop.load(Ordering::Relaxed) || started.elapsed() >= timeout {
            break Err("diagnostic command timed out or stopped".into());
        }
        match child.try_wait() {
            Ok(Some(status)) if !status.success() => break Err(format!("exited with {status}")),
            Ok(Some(_)) => match rx.recv_timeout(Duration::from_millis(40)) {
                Ok((Ok(_), bytes)) if bytes.len() <= OUTPUT_LIMIT => {
                    break Ok(String::from_utf8_lossy(&bytes).into_owned());
                }
                _ => break Err("diagnostic output unavailable or too large".into()),
            },
            Ok(None) => thread::sleep(Duration::from_millis(10)),
            Err(error) => break Err(error.to_string()),
        }
    };
    // Also close bounded readers if the helper failed or filled its output pipe.
    let _ = child.kill();
    let _ = child.wait();
    result
}

struct Stream {
    child: Child,
    lines: mpsc::Receiver<String>,
}
impl Stream {
    fn start(mut command: Command) -> Result<Self, String> {
        let mut child = command
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|e| e.to_string())?;
        let mut stdout = child.stdout.take().unwrap();
        let (tx, lines) = mpsc::sync_channel(128);
        thread::spawn(move || {
            let mut buffer = [0; 4096];
            let mut line = Vec::new();
            let mut oversize = false;
            loop {
                match stdout.read(&mut buffer) {
                    Ok(0) | Err(_) => break,
                    Ok(count) => {
                        for &byte in &buffer[..count] {
                            if byte == b'\n' {
                                if !oversize {
                                    let _ =
                                        tx.try_send(String::from_utf8_lossy(&line).into_owned());
                                }
                                line.clear();
                                oversize = false;
                            } else if line.len() < 16 * 1024 {
                                line.push(byte);
                            } else {
                                oversize = true;
                            }
                        }
                    }
                }
            }
        });
        Ok(Self { child, lines })
    }
    fn alive(&mut self) -> bool {
        matches!(self.child.try_wait(), Ok(None))
    }
}
impl Drop for Stream {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[derive(Default, Debug, Clone)]
struct Radio {
    bssid: Option<String>,
    ssid: Option<String>,
    associated: bool,
    signal: Option<f64>,
    tx: Option<f64>,
    rx: Option<f64>,
    retries: Option<u64>,
    failed: Option<u64>,
    beacons: Option<u64>,
}
fn number(source: &str) -> Option<f64> {
    source
        .split_whitespace()
        .next()?
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
}
fn parse_radio(link: &str, stations: &str) -> Radio {
    let mut radio = Radio::default();
    for line in link.lines().map(str::trim) {
        if let Some(value) = line.strip_prefix("Connected to ") {
            radio.associated = true;
            radio.bssid = value.split_whitespace().next().map(str::to_ascii_lowercase);
        }
        if let Some((key, value)) = line.split_once(':') {
            match key.trim() {
                "SSID" => radio.ssid = Some(value.trim().into()),
                "signal" => radio.signal = number(value),
                "tx bitrate" => radio.tx = number(value),
                "rx bitrate" => radio.rx = number(value),
                _ => {}
            }
        }
    }
    let mut matched = false;
    for line in stations.lines().map(str::trim) {
        if let Some(station) = line.strip_prefix("Station ") {
            matched = station
                .split_whitespace()
                .next()
                .is_some_and(|mac| Some(mac.to_ascii_lowercase()) == radio.bssid);
        } else if matched && let Some((key, value)) = line.split_once(':') {
            match key.trim() {
                "signal" if radio.signal.is_none() => radio.signal = number(value),
                "signal avg" => radio.signal = number(value).or(radio.signal),
                "tx bitrate" => radio.tx = number(value),
                "rx bitrate" => radio.rx = number(value),
                "tx retries" => radio.retries = value.trim().parse().ok(),
                "tx failed" => radio.failed = value.trim().parse().ok(),
                "beacon loss" => radio.beacons = value.trim().parse().ok(),
                _ => {}
            }
        }
    }
    radio
}
fn route_interface(output: &str) -> Option<String> {
    let interface = if output.trim_start().starts_with('[') {
        serde_json::from_str::<serde_json::Value>(output)
            .ok()?
            .as_array()?
            .first()?
            .get("dev")?
            .as_str()?
            .to_owned()
    } else {
        let words: Vec<_> = output.split_whitespace().collect();
        if words
            .iter()
            .any(|word| matches!(*word, "unreachable" | "prohibit" | "blackhole"))
        {
            return None;
        }
        words
            .windows(2)
            .find(|pair| pair[0] == "dev")?
            .get(1)?
            .to_string()
    };
    (!interface.is_empty()
        && interface.len() < 64
        && !interface.contains('/')
        && !interface.chars().any(char::is_whitespace))
    .then_some(interface)
}
fn event_message(source: &str, line: &str, interface: &str) -> Option<(EventKind, String)> {
    let message = if source == "NetworkManager" {
        let record: serde_json::Value = serde_json::from_str(line).ok()?;
        let message = record.get("MESSAGE")?.as_str()?;
        if let Some(device) = record.get("NM_DEVICE").and_then(|device| device.as_str()) {
            if device != interface {
                return None;
            }
        } else if !mentions_interface(message, interface) {
            return None;
        }
        message.to_owned()
    } else {
        if !mentions_interface(line, interface) {
            return None;
        }
        line.to_owned()
    };
    let text = message.to_ascii_lowercase();
    let kind = if text.contains("state change:") {
        match text.split(" -> ").nth(1)?.split_whitespace().next()? {
            "disconnected" | "failed" | "unavailable" => EventKind::Disconnect,
            "activated" => EventKind::Reconnect,
            "need-auth" => EventKind::Authentication,
            _ => return None,
        }
    } else if text.contains("deauth") || text.contains("disassoc") {
        EventKind::Deauthentication
    } else if text.contains("disconnected")
        || text.contains("link is not ready")
        || text.contains("-> failed")
        || text.contains("failed to connect")
        || text.contains("activation: failed")
        || text.contains("connection timed out")
    {
        EventKind::Disconnect
    } else if text.contains("beacon loss") || text.contains("beacon-loss") {
        EventKind::BeaconLoss
    } else if text.contains("connected to")
        || text.contains("roamed")
        || text.contains("-> activated")
        || text.contains("-> completed")
    {
        EventKind::Reconnect
    } else if text.contains("authenticat") || text.contains(" auth ") {
        EventKind::Authentication
    } else {
        return None;
    };
    Some((
        kind,
        format!(
            "{source}: {}",
            message.chars().take(300).collect::<String>()
        ),
    ))
}
fn mentions_interface(message: &str, interface: &str) -> bool {
    message
        .split(|ch: char| !(ch.is_ascii_alphanumeric() || matches!(ch, '_' | '-' | '.')))
        .any(|word| word == interface)
}
struct Sample {
    at: Instant,
    radio: Radio,
}
struct Worker {
    tools: Tools,
    stop: Arc<AtomicBool>,
    view: Snapshot,
    samples: VecDeque<Sample>,
    streams: Vec<(&'static str, Stream)>,
    stream_notes: Vec<String>,
    next_stream: Instant,
}
impl Worker {
    fn new(tools: Tools, stop: Arc<AtomicBool>) -> Self {
        Self {
            tools,
            stop,
            view: Snapshot::default(),
            samples: VecDeque::new(),
            streams: vec![],
            stream_notes: vec![],
            next_stream: Instant::now(),
        }
    }
    fn destination(&self, target: &Target) -> Result<IpAddr, String> {
        if let Some(peer) = target.peer {
            return Ok(peer);
        }
        if let Ok(address) = target.address.parse::<SocketAddr>() {
            return Ok(address.ip());
        }
        let host = target
            .address
            .rsplit_once(':')
            .map(|(host, _)| host)
            .ok_or("Invalid controller address")?;
        let output = self.tools.read("getent", &["ahosts", host], &self.stop)?;
        output
            .split_whitespace()
            .find_map(|word| word.parse().ok())
            .ok_or("Could not resolve controller address".into())
    }
    fn route(&mut self, target: &Target) {
        let result: Result<String, String> = (|| {
            let destination = self.destination(target)?;
            self.view.destination = Some(destination);
            let address = destination.to_string();
            self.tools
                .read("ip", &["-j", "route", "get", &address], &self.stop)
                .and_then(|output| {
                    route_interface(&output).ok_or("No interface in controller route".into())
                })
                .or_else(|_| {
                    self.tools
                        .read("ip", &["route", "get", &address], &self.stop)
                        .and_then(|output| {
                            route_interface(&output)
                                .ok_or("No interface in controller route".into())
                        })
                })
        })();
        match result {
            Ok(interface) => {
                if self.view.interface.as_ref() != Some(&interface) {
                    self.view = Snapshot {
                        destination: self.view.destination,
                        ..Default::default()
                    };
                    self.samples.clear();
                    self.streams.clear();
                    self.next_stream = Instant::now();
                }
                self.view.route_current = true;
                let sysfs_wifi = Path::new("/sys/class/net")
                    .join(&interface)
                    .join("phy80211")
                    .exists();
                let info = self
                    .tools
                    .read("iw", &["dev", &interface, "info"], &self.stop);
                self.view.kind = if sysfs_wifi || info.as_ref().is_ok_and(|s| s.contains("wiphy")) {
                    Kind::Wireless
                } else if info.is_err() && !Path::new("/sys/class/net").join(&interface).exists() {
                    Kind::Unavailable
                } else {
                    Kind::Other
                };
                if let Err(error) = info
                    && self.view.kind == Kind::Unavailable
                {
                    self.view.notes.push(error);
                }
                self.view.interface = Some(interface);
                if self.view.kind != Kind::Wireless {
                    self.streams.clear();
                    self.view.clear_radio();
                }
            }
            Err(error) => {
                self.view.route_current = false;
                if self.view.interface.is_none() {
                    self.view.kind = Kind::Unavailable;
                }
                self.view.notes.push(format!("Route unavailable: {error}"));
            }
        }
    }
    fn streams(&mut self) {
        self.streams.retain_mut(|(name, stream)| {
            if stream.alive() {
                true
            } else {
                self.stream_notes
                    .push(format!("{name} event listener stopped; will retry"));
                false
            }
        });
        if self.streams.len() < 2 && Instant::now() >= self.next_stream {
            self.next_stream = Instant::now() + Duration::from_secs(30);
            self.stream_notes.clear();
            for source in ["iw", "NetworkManager"] {
                if self.streams.iter().any(|(name, _)| *name == source) {
                    continue;
                }
                let command = if source == "iw" {
                    // iw on older systems may fully buffer stdout when piped.
                    self.tools.path("iw").and_then(|iw| {
                        self.tools
                            .command("stdbuf", &["-oL", &iw.to_string_lossy(), "event", "-t"])
                    })
                } else {
                    self.tools
                        .read(
                            "journalctl",
                            &[
                                "-n",
                                "1",
                                "-u",
                                "NetworkManager",
                                "-o",
                                "json",
                                "--no-pager",
                            ],
                            &self.stop,
                        )
                        .and_then(|output| {
                            if !output
                                .lines()
                                .any(|line| serde_json::from_str::<serde_json::Value>(line).is_ok())
                            {
                                return Err("No readable NetworkManager journal records".into());
                            }
                            self.tools.command(
                                "journalctl",
                                &[
                                    "-f",
                                    "-n",
                                    "0",
                                    "-u",
                                    "NetworkManager",
                                    "-o",
                                    "json",
                                    "--no-pager",
                                ],
                            )
                        })
                };
                match command.and_then(Stream::start) {
                    Ok(stream) => self.streams.push((source, stream)),
                    Err(error) => self
                        .stream_notes
                        .push(format!("{source} events unavailable: {error}")),
                }
            }
        }
        self.view.event_sources = self.streams.len();
        if let Some(interface) = self.view.interface.clone() {
            for (source, stream) in &self.streams {
                for line in stream.lines.try_iter().take(128) {
                    if let Some((kind, text)) = event_message(source, &line, &interface) {
                        self.view.event(kind, text);
                    }
                }
            }
        }
    }
    fn sample(&mut self) {
        let interface = self.view.interface.as_deref().unwrap();
        let link = self
            .tools
            .read("iw", &["dev", interface, "link"], &self.stop);
        let stations = self
            .tools
            .read("iw", &["dev", interface, "station", "dump"], &self.stop);
        self.view.clear_radio();
        match link {
            Ok(link) if link.contains("Connected to ") || link.contains("Not connected") => {
                let radio = parse_radio(&link, stations.as_deref().unwrap_or(""));
                if let Err(error) = stations {
                    self.view.notes.push(error);
                }
                if let Some(previous) = self.samples.back() {
                    if previous.radio.associated && !radio.associated {
                        self.view
                            .event(EventKind::Disconnect, "iw polling: association lost".into());
                    } else if !previous.radio.associated && radio.associated {
                        self.view
                            .event(EventKind::Reconnect, "iw polling: associated again".into());
                    } else if previous.radio.associated
                        && radio.associated
                        && previous.radio.bssid != radio.bssid
                    {
                        self.view.event(
                            EventKind::Reconnect,
                            "iw polling: access point changed".into(),
                        );
                    }
                }
                let reset = self.samples.back().is_some_and(|previous| {
                    previous.radio.bssid != radio.bssid
                        || [
                            (previous.radio.retries, radio.retries),
                            (previous.radio.failed, radio.failed),
                            (previous.radio.beacons, radio.beacons),
                        ]
                        .iter()
                        .any(|(a, b)| matches!((a,b), (Some(a),Some(b)) if b<a))
                });
                if reset {
                    self.samples.clear();
                }
                let now = Instant::now();
                self.samples.push_back(Sample {
                    at: now,
                    radio: radio.clone(),
                });
                while self.samples.len() > 2 && now.duration_since(self.samples[1].at) > WINDOW {
                    self.samples.pop_front();
                }
                if let Some(first) = self.samples.front() {
                    let seconds = now.duration_since(first.at).as_secs_f64();
                    if seconds > 0.1 {
                        self.view.window_seconds = Some(seconds);
                        self.view.retries_per_second = radio
                            .retries
                            .zip(first.radio.retries)
                            .and_then(|(a, b)| a.checked_sub(b))
                            .map(|delta| delta as f64 / seconds);
                        self.view.failures_per_second = radio
                            .failed
                            .zip(first.radio.failed)
                            .and_then(|(a, b)| a.checked_sub(b))
                            .map(|delta| delta as f64 / seconds);
                        self.view.beacon_losses = radio
                            .beacons
                            .zip(first.radio.beacons)
                            .and_then(|(a, b)| a.checked_sub(b));
                    }
                }
                self.view.associated = Some(radio.associated);
                self.view.ssid = radio.ssid;
                self.view.bssid = radio.bssid;
                self.view.signal = radio.signal;
                self.view.tx_bitrate = radio.tx;
                self.view.rx_bitrate = radio.rx;
                self.view.sampled_at = Some(now);
            }
            Ok(_) => {
                self.samples.clear();
                self.view.notes.push("Unrecognized iw link output".into());
            }
            Err(error) => {
                self.samples.clear();
                self.view.notes.push(error);
            }
        }
    }
    fn run(mut self, shared: Arc<Mutex<Snapshot>>, target: Arc<Mutex<Target>>) {
        let mut current: Option<Target> = None;
        let mut next_route = Instant::now();
        let mut next_sample = Instant::now();
        let mut route_note = None;
        while !self.stop.load(Ordering::Relaxed) {
            let next_target = target.lock().unwrap().clone();
            if current.as_ref() != Some(&next_target) {
                self.view = Snapshot::default();
                self.samples.clear();
                self.streams.clear();
                self.next_stream = Instant::now();
                next_route = Instant::now();
                next_sample = Instant::now();
                current = Some(next_target.clone());
            }
            if Instant::now() >= next_route {
                self.view.notes.clear();
                self.route(&next_target);
                route_note = self.view.notes.first().cloned();
                next_route = Instant::now() + Duration::from_secs(5);
            }
            if self.view.kind == Kind::Wireless {
                self.streams();
                if Instant::now() >= next_sample {
                    self.view.notes = route_note.iter().cloned().collect();
                    self.sample();
                    self.view.notes.extend(self.stream_notes.iter().cloned());
                    next_sample = Instant::now() + Duration::from_secs(1);
                }
            }
            self.view.controller_address = next_target.address;
            *shared.lock().unwrap() = self.view.clone();
            thread::sleep(Duration::from_millis(50));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{fs, os::unix::fs::PermissionsExt, sync::atomic::AtomicU64};
    const LINK: &str = "Connected to aa:bb:cc:dd:ee:ff (on oldwifi0)\n\tSSID: Plasma controller\n\tsignal: -67 dBm\n\ttx bitrate: 54.0 MBit/s\n";
    const STATION: &str = "Station 11:22:33:44:55:66 (on oldwifi0)\n\ttx retries: 99999\nStation aa:bb:cc:dd:ee:ff (on oldwifi0)\n\tsignal avg: -69 [-68, -70] dBm\n\ttx bitrate: 72.2 MBit/s MCS 7\n\trx bitrate: 24.0 MBit/s\n\ttx retries: 100\n\ttx failed: 4\n\tbeacon loss: 3\n";
    static NEXT: AtomicU64 = AtomicU64::new(0);
    struct Fixture(PathBuf);
    impl Fixture {
        fn new() -> Self {
            let path = std::env::temp_dir().join(format!(
                "plasma-wifi-diag-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir_all(&path).unwrap();
            Self(path)
        }
        fn script(&self, name: &str, body: &str) -> PathBuf {
            let path = self.0.join(name);
            fs::write(&path, format!("#!/bin/sh\n{body}\n")).unwrap();
            fs::set_permissions(&path, fs::Permissions::from_mode(0o755)).unwrap();
            path
        }
        fn tools(&self) -> Tools {
            let ip = self.script("ip", "if [ \"$1\" = '-j' ]; then exit 2; fi\n[ \"$3\" = '192.168.0.42' ] || exit 3\nprintf '192.168.0.42 dev oldwifi0 src 192.168.0.2\\n'");
            let iw = self.script("iw", &format!("case \"$3\" in\n info) echo 'wiphy 0';;\n link) /bin/cat '{}/link';;\n station) /bin/cat '{}/station';;\n *) exit 2;;\nesac", self.0.display(), self.0.display()));
            fs::write(self.0.join("link"), LINK).unwrap();
            fs::write(self.0.join("station"), STATION).unwrap();
            Tools {
                paths: vec![("ip", Some(ip)), ("iw", Some(iw))],
            }
        }
    }
    impl Drop for Fixture {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }
    fn target() -> Target {
        Target {
            address: "192.168.0.42:23".into(),
            peer: None,
        }
    }

    #[test]
    fn routes_support_old_ip_output_ipv6_and_json_without_guessing_an_interface() {
        assert_eq!(
            route_interface(r#"[{"dst":"192.168.0.1","dev":"wlan9","prefsrc":"192.168.0.2"}]"#)
                .as_deref(),
            Some("wlan9")
        );
        assert_eq!(
            route_interface("192.168.0.1 via 10.0.0.1 dev eth2 src 10.0.0.2 uid 1000\n cache")
                .as_deref(),
            Some("eth2")
        );
        assert_eq!(
            route_interface("fd00::1 from :: dev wifi-old metric 100").as_deref(),
            Some("wifi-old")
        );
        for output in [
            "RTNETLINK answers: Network is unreachable",
            "unreachable 192.168.0.1 dev lo",
            "[]",
            "[{\"dev\":\"../../other\"}]",
            "bad output",
        ] {
            assert!(route_interface(output).is_none(), "{output}");
        }
    }
    #[test]
    fn station_fields_are_optional_and_only_the_associated_ap_is_used() {
        let radio = parse_radio(LINK, STATION);
        assert_eq!(radio.signal, Some(-69.0));
        assert_eq!(radio.tx, Some(72.2));
        assert_eq!(radio.rx, Some(24.0));
        assert_eq!(radio.retries, Some(100));
        assert_eq!(radio.failed, Some(4));
        assert_eq!(radio.beacons, Some(3));
        let old = parse_radio(LINK, "");
        assert_eq!(old.tx, Some(54.0));
        assert_eq!(old.rx, None);
        assert_eq!(old.retries, None);
        assert!(!parse_radio("Not connected.\n", STATION).associated);
    }
    #[test]
    fn events_filter_devices_parse_transition_destinations_and_merge_duplicates() {
        assert!(event_message("iw", "1.00: oldwifi01: disconnected", "oldwifi0").is_none());
        assert!(
            event_message(
                "NetworkManager",
                r#"{"NM_DEVICE":"eth0","MESSAGE":"oldwifi0 disconnected"}"#,
                "oldwifi0"
            )
            .is_none()
        );
        assert!(event_message("NetworkManager", r#"{"MESSAGE":"device (oldwifi0): state change: disconnected -> prepare (reason none)"}"#, "oldwifi0").is_none());
        let (kind, message) =
            event_message("iw", "1.00: oldwifi0 (phy #0): deauth reason 4", "oldwifi0").unwrap();
        assert_eq!(kind, EventKind::Deauthentication);
        let mut view = Snapshot {
            event_sources: 1,
            ..Default::default()
        };
        view.event(kind, message);
        let (kind, message) = event_message("NetworkManager", r#"{"MESSAGE":"device (oldwifi0): state change: activated -> disconnected (reason supplicant-disconnect)"}"#, "oldwifi0").unwrap();
        view.event(kind, message);
        assert_eq!(view.event_count(EventKind::Disconnect), Some(1));
        assert_eq!(view.event_count(EventKind::Deauthentication), Some(1));
        view.event(
            EventKind::Deauthentication,
            "iw: another deauth reason 4".into(),
        );
        assert_eq!(
            view.event_count(EventKind::Deauthentication),
            Some(2),
            "Repeated events from the same observer must remain visible"
        );
        assert_eq!(
            event_message(
                "iw",
                "1.02: oldwifi0: connected to aa:bb:cc:dd:ee:ff",
                "oldwifi0"
            )
            .unwrap()
            .0,
            EventKind::Reconnect
        );
        assert_eq!(
            event_message(
                "iw",
                "1.02: oldwifi0: CQM event: beacon loss detected",
                "oldwifi0"
            )
            .unwrap()
            .0,
            EventKind::BeaconLoss
        );
        assert_eq!(Snapshot::default().event_count(EventKind::Disconnect), None);
    }
    #[test]
    fn legacy_commands_produce_rates_and_counter_resets_start_a_new_baseline() {
        let fixture = Fixture::new();
        let mut worker = Worker::new(fixture.tools(), Arc::new(AtomicBool::new(false)));
        worker.route(&target());
        assert_eq!(worker.view.kind, Kind::Wireless);
        assert_eq!(worker.view.interface.as_deref(), Some("oldwifi0"));
        worker.sample();
        assert_eq!(worker.view.retries_per_second, None);
        worker.samples.front_mut().unwrap().at = Instant::now() - Duration::from_secs(2);
        fs::write(
            fixture.0.join("station"),
            STATION
                .replace("retries: 100", "retries: 120")
                .replace("failed: 4", "failed: 6")
                .replace("loss: 3", "loss: 4"),
        )
        .unwrap();
        worker.sample();
        assert!((9.0..11.0).contains(&worker.view.retries_per_second.unwrap()));
        assert!((0.9..1.1).contains(&worker.view.failures_per_second.unwrap()));
        assert_eq!(worker.view.beacon_losses, Some(1));
        fs::write(fixture.0.join("station"), STATION).unwrap();
        worker.sample();
        assert_eq!(worker.view.retries_per_second, None);
        assert_eq!(worker.samples.len(), 1);
        fs::write(fixture.0.join("link"), "Not connected.\n").unwrap();
        worker.sample();
        assert_eq!(worker.view.signal, None);
        assert_eq!(worker.view.associated, Some(false));
        assert!(
            worker
                .view
                .events
                .iter()
                .any(|event| event.kind == EventKind::Disconnect)
        );
        fs::write(fixture.0.join("link"), LINK).unwrap();
        worker.sample();
        assert!(
            worker
                .view
                .events
                .iter()
                .any(|event| event.kind == EventKind::Reconnect)
        );
        worker.streams();
        assert_eq!(worker.view.event_sources, 0);
        assert_eq!(worker.stream_notes.len(), 2);
    }
    #[test]
    fn missing_tools_and_lost_routes_preserve_app_operation_and_identify_last_device() {
        let fixture = Fixture::new();
        let tools = fixture.tools();
        let mut worker = Worker::new(tools.clone(), Arc::new(AtomicBool::new(false)));
        worker.route(&target());
        fs::remove_file(tools.path("ip").unwrap()).unwrap();
        worker.route(&target());
        assert!(!worker.view.route_current);
        assert_eq!(worker.view.interface.as_deref(), Some("oldwifi0"));
        worker.sample();
        assert_eq!(worker.view.signal, Some(-69.0));
        let mut absent = Worker::new(Tools { paths: vec![] }, worker.stop.clone());
        absent.route(&target());
        assert_eq!(absent.view.kind, Kind::Unavailable);
        assert!(absent.view.notes[0].contains("not installed"));
    }
    #[test]
    fn helpers_have_time_and_output_limits_and_monitor_snapshots_never_wait_on_helpers() {
        let stop = AtomicBool::new(false);
        let mut slow = Command::new("/bin/sleep");
        slow.arg("10");
        let started = Instant::now();
        assert!(read_command(slow, &stop, Duration::from_millis(60)).is_err());
        assert!(started.elapsed() < Duration::from_millis(500));
        assert!(read_command(Command::new("/usr/bin/yes"), &stop, Duration::from_secs(1)).is_err());
        let fixture = Fixture::new();
        let mut tools = fixture.tools();
        tools.paths[0].1 = Some(fixture.script("hanging-ip", "exec /bin/sleep 10"));
        let shared = Arc::new(Mutex::new(Snapshot::default()));
        let target = Arc::new(Mutex::new(target()));
        let stop = Arc::new(AtomicBool::new(false));
        let (thread_shared, thread_target, thread_stop) =
            (shared.clone(), target.clone(), stop.clone());
        let monitor = Monitor {
            shared,
            target,
            stop,
            handle: Some(thread::spawn(move || {
                Worker::new(tools, thread_stop).run(thread_shared, thread_target)
            })),
        };
        thread::sleep(Duration::from_millis(30));
        let started = Instant::now();
        for _ in 0..100 {
            monitor.snapshot("192.168.0.42:23", None);
        }
        assert!(started.elapsed() < Duration::from_millis(100));
        drop(monitor);
        assert!(started.elapsed() < Duration::from_millis(500));
    }
    #[test]
    fn demo_never_starts_helpers_and_retargeting_does_not_show_old_readings() {
        let monitor = Monitor::start("192.168.0.42:23".into(), true);
        assert!(monitor.handle.is_none());
        assert_eq!(monitor.snapshot("192.168.0.42:23", None).kind, Kind::Demo);
        assert_eq!(
            monitor.snapshot("192.168.0.43:23", None).kind,
            Kind::Checking
        );
        assert_eq!(
            monitor.snapshot("192.168.0.43:23", None).kind,
            Kind::Checking
        );
    }

    #[test]
    fn missing_diagnostics_do_not_prevent_a_controller_job() {
        use crate::controller::{Action, Controller};
        let simulator = crate::simulator::Simulator::start().unwrap();
        let shared = Arc::new(Mutex::new(Snapshot::default()));
        let target = Arc::new(Mutex::new(Target {
            address: simulator.address.clone(),
            peer: None,
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let (thread_shared, thread_target, thread_stop) =
            (shared.clone(), target.clone(), stop.clone());
        let monitor = Monitor {
            shared,
            target,
            stop,
            handle: Some(thread::spawn(move || {
                Worker::new(Tools { paths: vec![] }, thread_stop).run(thread_shared, thread_target);
            })),
        };
        let controller = Controller::start(simulator.address.clone(), simulator.http.clone(), None);
        controller.send(Action::Load {
            name: "no-diagnostics.nc".into(),
            text: "G21G90G94\nG1X1F600\nM30\n".into(),
        });
        let deadline = Instant::now() + Duration::from_secs(10);
        while !controller.snapshot().can_run() {
            assert!(
                Instant::now() < deadline,
                "Controller initialization blocked by unavailable diagnostics"
            );
            thread::sleep(Duration::from_millis(20));
        }
        controller.send(Action::Run);
        loop {
            let status = controller.snapshot();
            if !status.job && status.progress == 1.0 {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "Job blocked by unavailable diagnostics"
            );
            thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(
            monitor.snapshot(&simulator.address, None).kind,
            Kind::Unavailable
        );
    }
}
