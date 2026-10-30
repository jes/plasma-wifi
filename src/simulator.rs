//! Loopback-only FluidNC simulator for repeatable UI and protocol checks.
use crate::gcode::{self, ParseOptions, Point, Program};
use anyhow::{Context, Result};
use serde_json::json;
use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

pub const DEMO: &str = "(Plasma WiFi example: rounded plate and circular cutout)\nG21 G90 G17 G94\nG54\nG0 X10 Y10\nM3 S1\nG1 X80 Y10 F1200\nG3 X90 Y20 I0 J10\nG1 Y60\nG3 X80 Y70 I-10 J0\nG1 X20\nG3 X10 Y60 I0 J-10\nG1 Y10\nM5\nG0 X60 Y40\nM3 S1\nG3 X60 Y40 I-10 J0 F900\nM5\nG0 X0 Y0\nM30\n";

#[derive(Default)]
struct Machine {
    file: String,
    program: Option<Program>,
    block: usize,
    block_seconds: f64,
    position: Point,
    wco: Point,
    torch: bool,
    held: bool,
    hold_until: Option<Instant>,
    alarm: Option<u8>,
    override_stopped: bool,
    feed: f64,
    rapid: f64,
    complete: bool,
    jog_until: Option<Instant>,
    inches: bool,
    modal_inches: bool,
    reboot_banner: bool,
    reject_offsets: bool,
    ignore_offsets: bool,
    disconnect_generation: u64,
    pub commands: Vec<String>,
    report_count: u64,
    report_wco: Point,
    report_feed: f64,
    report_torch: bool,
}
impl Machine {
    fn state(&self) -> &str {
        if self.alarm.is_some() {
            "Alarm"
        } else if self.held {
            if self.hold_until.is_some_and(|until| until > Instant::now()) {
                "Hold:1"
            } else {
                "Hold:0"
            }
        } else if self.jog_until.is_some_and(|t| t > Instant::now()) {
            "Jog"
        } else if self.program.is_some() {
            "Run"
        } else {
            "Idle"
        }
    }
    fn status(&mut self, auto: bool) -> String {
        let periodic = self.report_count.is_multiple_of(10);
        self.report_count += 1;
        let wco = if periodic || auto && self.report_wco != self.wco {
            self.report_wco = self.wco;
            let scale = if self.inches { 25.4 } else { 1.0 };
            format!(
                "|WCO:{:.4},{:.4},{:.4}",
                self.wco.x / scale,
                self.wco.y / scale,
                self.wco.z / scale
            )
        } else {
            String::new()
        };
        let overrides = if periodic
            || auto && (self.report_feed != self.feed || self.report_torch != self.torch)
        {
            self.report_feed = self.feed;
            self.report_torch = self.torch;
            format!(
                "|Ov:{:.0},{:.0},100{}",
                self.feed * 100.0,
                self.rapid * 100.0,
                if self.torch { "|A:S" } else { "" }
            )
        } else {
            String::new()
        };
        let line = self
            .program
            .as_ref()
            .and_then(|p| p.blocks.get(self.block))
            .map(|b| format!("|Ln:{}", b.n))
            .unwrap_or_default();
        let file = if let Some(p) = &self.program {
            format!(
                "|SD:{:.2},/localfs/plasma.nc",
                self.block as f64 / p.blocks.len() as f64 * 100.0
            )
        } else {
            String::new()
        };
        let scale = if self.inches { 25.4 } else { 1.0 };
        format!(
            "<{}|MPos:{:.4},{:.4},{:.4}{wco}{line}|FS:{},{}{overrides}{file}>\n",
            self.state(),
            (self.position.x + self.wco.x) / scale,
            (self.position.y + self.wco.y) / scale,
            (self.position.z + self.wco.z) / scale,
            if self.state() == "Run" { 1200 } else { 0 },
            if self.torch { 1 } else { 0 },
        )
    }
    fn tick(&mut self, dt: f64) {
        if self.held {
            return;
        }
        let Some(p) = &self.program else {
            return;
        };
        let Some(block) = p.blocks.get(self.block) else {
            self.program = None;
            self.torch = false;
            self.complete = true;
            return;
        };
        if self.block_seconds == 0.0 {
            self.torch = block.torch;
        }
        let duration = block.time.seconds(self.feed, self.rapid, 1.0);
        self.block_seconds += dt;
        if !block.path.is_empty() {
            let t = if duration > 0.0 {
                (self.block_seconds / duration).min(1.0)
            } else {
                1.0
            };
            let f = t * (block.path.len() - 1) as f64;
            let i = (f.floor() as usize).min(block.path.len() - 1);
            self.position =
                block.path[i].lerp(block.path[(i + 1).min(block.path.len() - 1)], f - i as f64);
        }
        if self.block_seconds >= duration {
            self.block += 1;
            self.block_seconds = 0.0;
        }
    }
    fn command(&mut self, command: &str) -> String {
        self.commands.push(command.into());
        if command == "$X" {
            self.alarm = None;
            return "ok\n".into();
        }
        if command == "$report_inches" {
            return format!("$/report_inches={}\nok\n", self.inches);
        }
        if command == "$Report/Inches" {
            if self.program.is_some() || self.held {
                return "error:8\n".into();
            }
            return format!("$13={}\nok\n", self.inches as u8);
        }
        if command == "$I" {
            return "[VER:4.1:FluidNC v4.1.1 (simulator)]\nok\n".into();
        }
        if command == "$G" {
            return format!(
                "[GC:G0 G54 G17 {} G90 G94 M5 M9 T0 F0 S0]\nok\n",
                if self.modal_inches { "G20" } else { "G21" }
            );
        }
        if command == "$use_line_numbers" {
            return "$/use_line_numbers=true\nok\n".into();
        }
        if command.starts_with("$axes/") {
            return format!("$/{}=3000\nok\n", command.trim_start_matches('$'));
        }
        if command == "$LocalFS/Run=plasma.nc" {
            match gcode::parse(
                &self.file,
                ParseOptions {
                    start: self.position,
                    wco: self.wco,
                    ..Default::default()
                },
            ) {
                Ok(p) => {
                    self.program = Some(p);
                    self.block = 0;
                    self.block_seconds = 0.0;
                    self.complete = false;
                    return String::new();
                }
                Err(_) => return "error:20\n".into(),
            }
        }
        if command.starts_with("$J=") {
            if self.program.is_some() || self.held {
                return "error:8\n".into();
            }
            if self.alarm.is_some() {
                return "error:9\n".into();
            }
            for axis in ['X', 'Y'] {
                if let Some(value) = word(command, axis) {
                    let coordinate = if axis == 'X' {
                        &mut self.position.x
                    } else {
                        &mut self.position.y
                    };
                    if command.contains("G90") {
                        *coordinate = value;
                    } else {
                        *coordinate += value;
                    }
                }
            }
            self.jog_until = Some(Instant::now() + Duration::from_millis(140));
        }
        if command == "G21" || command == "G21G54" {
            self.modal_inches = false;
        }
        if command == "G20" {
            self.modal_inches = true;
        }
        if command.starts_with("G10L20P1") {
            if self.reject_offsets {
                return "error:20\n".into();
            }
            if self.ignore_offsets {
                return "ok\n".into();
            }
            let scale = if self.modal_inches { 25.4 } else { 1.0 };
            if let Some(value) = word(command, 'X') {
                let value = value * scale;
                self.wco.x += self.position.x - value;
                self.position.x = value;
            }
            if let Some(value) = word(command, 'Y') {
                let value = value * scale;
                self.wco.y += self.position.y - value;
                self.position.y = value;
            }
            if let Some(value) = word(command, 'Z') {
                let value = value * scale;
                self.wco.z += self.position.z - value;
                self.position.z = value;
            }
        }
        "ok\n".into()
    }
}
fn word(s: &str, letter: char) -> Option<f64> {
    let rest = s.split_once(letter)?.1;
    let number: String = rest
        .chars()
        .take_while(|c| c.is_ascii_digit() || matches!(c, '.' | '+' | '-'))
        .collect();
    number.parse().ok()
}

pub struct Simulator {
    pub address: String,
    pub http: String,
    #[cfg(test)]
    machine: Arc<Mutex<Machine>>,
}
impl Simulator {
    pub fn start() -> Result<Self> {
        let telnet = TcpListener::bind("127.0.0.1:0")?;
        let http = TcpListener::bind("127.0.0.1:0")?;
        let address = telnet.local_addr()?.to_string();
        let http_url = format!("http://{}", http.local_addr()?);
        let machine = Arc::new(Mutex::new(Machine {
            feed: 1.0,
            rapid: 1.0,
            ..Default::default()
        }));
        let state = machine.clone();
        thread::spawn(move || {
            let mut last = Instant::now();
            loop {
                let now = Instant::now();
                state
                    .lock()
                    .unwrap()
                    .tick(now.duration_since(last).as_secs_f64());
                last = now;
                thread::sleep(Duration::from_millis(10));
            }
        });
        let state = machine.clone();
        thread::spawn(move || {
            for stream in telnet.incoming().flatten() {
                let state = state.clone();
                thread::spawn(move || {
                    let _ = telnet_client(stream, state);
                });
            }
        });
        let state = machine.clone();
        thread::spawn(move || {
            for stream in http.incoming().flatten() {
                let state = state.clone();
                thread::spawn(move || {
                    let _ = http_client(stream, state);
                });
            }
        });
        Ok(Self {
            address,
            http: http_url,
            #[cfg(test)]
            machine,
        })
    }
    #[cfg(test)]
    pub fn disconnect(&self) {
        self.machine.lock().unwrap().disconnect_generation += 1;
    }
    #[cfg(test)]
    pub fn commands(&self) -> Vec<String> {
        self.machine.lock().unwrap().commands.clone()
    }
    #[cfg(test)]
    fn reboot(&self, close_connection: bool) {
        let mut m = self.machine.lock().unwrap();
        // Firmware keeps G54 in NVS but resets its step counters on power-up.
        m.position = Point {
            x: -m.wco.x,
            y: -m.wco.y,
            z: -m.wco.z,
        };
        m.modal_inches = false;
        m.program = None;
        m.torch = false;
        m.held = false;
        m.jog_until = None;
        m.report_count = 0;
        if close_connection {
            m.disconnect_generation += 1;
        } else {
            m.reboot_banner = true;
        }
    }
}
fn telnet_client(mut stream: TcpStream, state: Arc<Mutex<Machine>>) -> Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(20)))?;
    stream.set_write_timeout(Some(Duration::from_secs(1)))?;
    let generation = {
        let mut m = state.lock().unwrap();
        m.report_count = 0;
        m.disconnect_generation
    };
    stream.write_all(b"Grbl 4.1 [FluidNC v4.1.1 simulator]\n")?;
    let mut buffer = [0u8; 2048];
    let mut line = Vec::new();
    let mut interval = None;
    let mut last_report = Instant::now();
    let mut last_report_state = String::new();
    loop {
        {
            let mut m = state.lock().unwrap();
            if m.disconnect_generation != generation {
                return Ok(());
            }
            if m.reboot_banner {
                m.reboot_banner = false;
                interval = None;
                stream.write_all(b"Grbl 4.1 [FluidNC v4.1.1 simulator]\n")?;
            }
            // FluidNC auto-reports periodically during motion, but in Idle only
            // reports changes. Explicit ? queries still produce sparse reports.
            if interval.is_some_and(|interval| {
                m.report_wco != m.wco
                    || m.report_feed != m.feed
                    || m.report_torch != m.torch
                    || m.state() != last_report_state
                    || matches!(m.state(), "Run" | "Jog" | "Hold:1")
                        && last_report.elapsed() >= interval
            }) {
                stream.write_all(m.status(interval.is_some()).as_bytes())?;
                last_report_state = m.state().into();
                last_report = Instant::now();
            }
            if m.complete {
                stream.write_all(b"ok\n")?;
                m.complete = false;
            }
        }
        let count = match stream.read(&mut buffer) {
            Ok(0) => return Ok(()),
            Ok(n) => n,
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(e) => return Err(e.into()),
        };
        for &byte in &buffer[..count] {
            let mut m = state.lock().unwrap();
            match byte {
                b'?' => stream.write_all(m.status(interval.is_some()).as_bytes())?,
                b'!' => {
                    if matches!(m.state(), "Run" | "Jog") {
                        m.held = true;
                        m.hold_until = Some(Instant::now() + Duration::from_millis(100));
                    }
                }
                b'~' => {
                    m.held = false;
                }
                0x9e => {
                    if m.held {
                        m.override_stopped = !m.override_stopped;
                        m.torch = !m.override_stopped
                            && m.program
                                .as_ref()
                                .and_then(|p| p.blocks.get(m.block))
                                .is_some_and(|b| b.torch);
                    }
                }
                0x18 => {
                    let abort = matches!(m.state(), "Run" | "Jog" | "Hold:1");
                    if abort {
                        m.alarm = Some(3);
                    }
                    m.program = None;
                    m.held = false;
                    m.torch = false;
                    m.override_stopped = false;
                    m.jog_until = None;
                    line.clear();
                    interval = None;
                    m.report_count = 0;
                    stream.write_all(b"Grbl 4.1 [FluidNC v4.1.1 simulator]\n")?;
                    if abort {
                        stream.write_all(b"ALARM:3\n")?;
                    }
                }
                0x85 => {
                    m.jog_until = None;
                }
                0x90 => m.feed = 1.0,
                0x91 => m.feed = (m.feed + 0.1).min(2.0),
                0x92 => m.feed = (m.feed - 0.1).max(0.1),
                0x93 => m.feed = (m.feed + 0.01).min(2.0),
                0x94 => m.feed = (m.feed - 0.01).max(0.1),
                0x99 => {}
                b'\n' => {
                    let command = String::from_utf8_lossy(&line).trim().to_owned();
                    line.clear();
                    if let Some(value) = command
                        .strip_prefix("$Report/Interval=")
                        .and_then(|v| v.parse::<u64>().ok())
                    {
                        interval = (value > 0).then(|| Duration::from_millis(value));
                    }
                    stream.write_all(m.command(&command).as_bytes())?;
                }
                b'\r' => {}
                _ => line.push(byte),
            }
        }
    }
}
fn http_client(mut stream: TcpStream, state: Arc<Mutex<Machine>>) -> Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(3)))?;
    let mut data = Vec::new();
    let mut buffer = [0u8; 4096];
    let header_end;
    loop {
        let n = stream.read(&mut buffer)?;
        if n == 0 {
            return Ok(());
        }
        data.extend_from_slice(&buffer[..n]);
        if let Some(i) = data.windows(4).position(|v| v == b"\r\n\r\n") {
            header_end = i + 4;
            break;
        }
        anyhow::ensure!(data.len() < 65536, "Headers too large");
    }
    let header = String::from_utf8_lossy(&data[..header_end]).into_owned();
    let length = header
        .lines()
        .find_map(|l| {
            let (k, v) = l.split_once(':')?;
            if k.eq_ignore_ascii_case("content-length") {
                v.trim().parse::<usize>().ok()
            } else {
                None
            }
        })
        .unwrap_or(0);
    anyhow::ensure!(length < 16 * 1024 * 1024, "Body too large");
    while data.len() < header_end + length {
        let n = stream.read(&mut buffer)?;
        if n == 0 {
            return Ok(());
        }
        data.extend_from_slice(&buffer[..n]);
    }
    let request = header.lines().next().context("Request line missing")?;
    let mut parts = request.split_whitespace();
    let method = parts.next().unwrap_or("");
    let path = parts.next().unwrap_or("");
    let mut m = state.lock().unwrap();
    let (code, content, body) = if path == "/sim/status" {
        (200,"application/json",serde_json::to_vec(&json!({"state":m.state(),"x":m.position.x,"y":m.position.y,"mpos_x":m.position.x+m.wco.x,"mpos_y":m.position.y+m.wco.y,"torch":m.torch,"feed_override":m.feed,"alarm":m.alarm,"commands":m.commands,"file":m.file})).unwrap())
    } else if path == "/sim/disconnect" && method == "POST" {
        m.disconnect_generation += 1;
        (200, "application/json", b"{}".to_vec())
    } else if method == "POST" && path.starts_with("/files") {
        let boundary = header
            .lines()
            .find_map(|l| l.split_once("boundary=").map(|p| p.1.trim()))
            .context("Missing multipart boundary")?;
        let body = &data[header_end..];
        let start = body
            .windows(4)
            .position(|v| v == b"\r\n\r\n")
            .context("No multipart file")?
            + 4;
        let marker = format!("\r\n--{boundary}");
        let end = body[start..]
            .windows(marker.len())
            .position(|v| v == marker.as_bytes())
            .context("No multipart terminator")?
            + start;
        m.file = String::from_utf8(body[start..end].to_vec())?;
        (200, "application/json", listing(&m))
    } else if path.starts_with("/files") {
        if path.contains("action=delete") {
            m.file.clear();
        }
        (200, "application/json", listing(&m))
    } else if path == "/localfs/plasma.nc" {
        (200, "application/octet-stream", m.file.as_bytes().to_vec())
    } else {
        (404, "text/plain", b"Not found".to_vec())
    };
    stream.write_all(format!("HTTP/1.1 {code} OK\r\nContent-Type: {content}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",body.len()).as_bytes())?;
    stream.write_all(&body)?;
    Ok(())
}
fn listing(m: &Machine) -> Vec<u8> {
    serde_json::to_vec(&json!({"files":if m.file.is_empty(){vec![]}else{vec![json!({"name":"plasma.nc","size":m.file.len()})]},"total":"1.00 MB","used":format!("{} B",m.file.len()),"status":"Ok"})).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        controller::{Action, Controller, Snapshot},
        protocol,
    };
    fn wait(controller: &Controller, condition: impl Fn(&Snapshot) -> bool) -> Snapshot {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            let s = controller.snapshot();
            if condition(&s) {
                return s;
            }
            assert!(
                Instant::now() < deadline,
                "Timed out: state={} phase={} error={:?} upload={}",
                s.status.state,
                s.phase,
                s.error,
                s.upload
            );
            thread::sleep(Duration::from_millis(20));
        }
    }
    struct CheckpointFile(std::path::PathBuf);
    impl CheckpointFile {
        fn new() -> Self {
            static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let id = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            Self(std::env::temp_dir().join(format!(
                "plasma-wifi-{}-{id}/coordinates.json",
                std::process::id()
            )))
        }
        fn start(&self, sim: &Simulator) -> Controller {
            Controller::start(sim.address.clone(), sim.http.clone(), Some(self.0.clone()))
        }
        fn saved(&self, sim: &Simulator) -> Option<Point> {
            crate::coordinates::Coordinates::load(self.0.clone())
                .ok()?
                .get(&sim.address)
        }
    }
    impl Drop for CheckpointFile {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(self.0.parent().unwrap());
        }
    }
    fn seed_position(sim: &Simulator) -> Point {
        let mut m = sim.machine.lock().unwrap();
        m.position = Point {
            x: 81.25,
            y: -32.5,
            z: 7.125,
        };
        m.wco = Point {
            x: 10.0,
            y: 20.0,
            z: 3.0,
        };
        m.position
    }
    fn restored(controller: &Controller, target: Point) -> Snapshot {
        wait(controller, |s| {
            s.initialized && s.status.position_known && s.status.wpos.distance(target) < 0.004
        })
    }
    #[test]
    fn coordinates_survive_every_app_and_firmware_restart_combination() {
        for (restart_app, reboot) in [(true, false), (true, true), (false, true), (false, false)] {
            let sim = Simulator::start().unwrap();
            let checkpoint = CheckpointFile::new();
            let target = seed_position(&sim);
            let controller = checkpoint.start(&sim);
            restored(&controller, target);
            // Simulate an origin edit followed by a jog: the saved current work
            // position must track both, rather than just remembering the origin.
            controller.send(Action::SetCoordinate {
                axis: 'X',
                value: 12.0,
            });
            wait(&controller, |s| {
                s.status.position_known && (s.status.wpos.x - 12.0).abs() < 0.001
            });
            controller.send(Action::Jog {
                step: 1.0,
                x: 1.0,
                y: 0.0,
                speed: 600.0,
            });
            wait(&controller, |s| s.status.state == "Jog");
            controller.send(Action::CancelJog);
            let target = wait(&controller, |s| s.status.idle()).status.wpos;
            // Snapshot publication follows the durable checkpoint write.
            assert!(checkpoint.saved(&sim).unwrap().distance(target) < 1e-9);
            let baseline = sim.commands().len();
            let controller = if restart_app {
                drop(controller);
                if reboot {
                    sim.reboot(true);
                }
                checkpoint.start(&sim)
            } else {
                if reboot {
                    sim.reboot(true);
                } else {
                    sim.disconnect();
                }
                wait(&controller, |s| !s.connected);
                controller
            };
            let expected_machine = {
                let m = sim.machine.lock().unwrap();
                Point {
                    x: m.position.x + m.wco.x,
                    y: m.position.y + m.wco.y,
                    z: m.position.z + m.wco.z,
                }
            };
            let s = restored(&controller, target);
            assert!(
                s.status.mpos.distance(expected_machine) < 0.004,
                "Restoring must not move the machine"
            );
            assert!(s.error.is_none(), "{:?}", s.error);
            let commands = sim.commands();
            let writes = commands[baseline..]
                .iter()
                .filter(|c| c.starts_with("G10"))
                .count();
            assert_eq!(writes, usize::from(reboot));
            assert!(!commands[baseline..].iter().any(|c| c.starts_with("$J=")
                || c.starts_with("$LocalFS/Run=")
                || c.starts_with("G0")
                || c.starts_with("G1") && !c.starts_with("G10")));
            // Repeat the reboot after a restoration to catch checkpoint drift.
            sim.reboot(true);
            wait(&controller, |s| !s.connected);
            restored(&controller, target);
        }
    }
    #[test]
    fn reboot_banner_on_existing_link_discards_cached_coordinates() {
        let sim = Simulator::start().unwrap();
        let checkpoint = CheckpointFile::new();
        let target = seed_position(&sim);
        let controller = checkpoint.start(&sim);
        restored(&controller, target);
        let count = sim.commands().len();
        sim.reboot(false);
        wait(&controller, |s| {
            sim.commands()[count..].iter().any(|c| c.starts_with("G10")) && s.initialized
        });
        let s = restored(&controller, target);
        assert!(s.status.mpos.distance(Point::default()) < 0.001);
        assert!(checkpoint.saved(&sim).unwrap().distance(target) < 1e-9);
    }
    #[test]
    fn restoration_with_sparse_idle_reports_enables_jogging_without_stop() {
        let sim = Simulator::start().unwrap();
        let checkpoint = CheckpointFile::new();
        let target = seed_position(&sim);
        let controller = checkpoint.start(&sim);
        restored(&controller, target);
        drop(controller);
        sim.reboot(true);
        let baseline = sim.commands().len();
        let controller = checkpoint.start(&sim);
        let s = restored(&controller, target);
        assert!(s.can_jog() && s.error.is_none());
        assert_ne!(s.phase, "Restoring work coordinates…");
        assert_eq!(
            sim.commands()[baseline..]
                .iter()
                .filter(|c| c.starts_with("G10"))
                .count(),
            1
        );
        assert!(s.status.mpos.distance(Point::default()) < 0.001);
    }
    #[test]
    fn inch_reports_and_modal_units_survive_coordinate_restoration() {
        let sim = Simulator::start().unwrap();
        let checkpoint = CheckpointFile::new();
        let target = seed_position(&sim);
        let controller = checkpoint.start(&sim);
        restored(&controller, target);
        drop(controller);
        sim.reboot(true);
        {
            let mut m = sim.machine.lock().unwrap();
            m.inches = true;
            m.modal_inches = true;
        }
        let controller = checkpoint.start(&sim);
        let s = restored(&controller, target);
        assert!(!s.options.initial_mm);
        assert!(sim.machine.lock().unwrap().modal_inches);
        assert!(s.status.mpos.distance(Point::default()) < 0.004);
    }
    #[test]
    fn alarm_waits_for_explicit_unlock_before_restoration() {
        let sim = Simulator::start().unwrap();
        let checkpoint = CheckpointFile::new();
        let target = seed_position(&sim);
        let controller = checkpoint.start(&sim);
        restored(&controller, target);
        drop(controller);
        sim.reboot(true);
        sim.machine.lock().unwrap().alarm = Some(3);
        let baseline = sim.commands().len();
        let controller = checkpoint.start(&sim);
        wait(&controller, |s| {
            s.status.state == "Alarm" && s.version.contains("FluidNC")
        });
        assert!(!controller.snapshot().initialized);
        assert!(
            !sim.commands()[baseline..]
                .iter()
                .any(|c| c == "$X" || c.starts_with("G10"))
        );
        assert!(checkpoint.saved(&sim).unwrap().distance(target) < 1e-9);
        controller.send(Action::Unlock);
        restored(&controller, target);
    }
    #[test]
    fn rejected_or_unverified_restoration_preserves_checkpoint_and_blocks_motion() {
        for rejected in [true, false] {
            let sim = Simulator::start().unwrap();
            let checkpoint = CheckpointFile::new();
            let target = seed_position(&sim);
            let controller = checkpoint.start(&sim);
            restored(&controller, target);
            drop(controller);
            sim.reboot(true);
            {
                let mut m = sim.machine.lock().unwrap();
                m.reject_offsets = rejected;
                m.ignore_offsets = !rejected;
            }
            let controller = checkpoint.start(&sim);
            let s = wait(&controller, |s| s.error.is_some());
            assert!(!s.initialized && !s.can_jog() && !s.can_run());
            assert!(checkpoint.saved(&sim).unwrap().distance(target) < 1e-9);
            assert!(s.status.mpos.distance(Point::default()) < 0.001);
        }
    }
    #[test]
    fn app_restart_during_running_job_keeps_live_coordinates() {
        let sim = Simulator::start().unwrap();
        let checkpoint = CheckpointFile::new();
        let controller = checkpoint.start(&sim);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::Load {
            name: "restart.nc".into(),
            text: "G21G90\nM3S1\nG1X100F60\nM5\nM30".into(),
        });
        wait(&controller, |s| s.can_run());
        controller.send(Action::Run);
        wait(&controller, |s| s.status.state == "Run");
        drop(controller);
        let baseline = sim.commands().len();
        let controller = checkpoint.start(&sim);
        let s = wait(&controller, |s| s.initialized && s.status.state == "Run");
        assert!(!s.can_jog() && !s.can_run());
        assert!(
            !sim.commands()[baseline..]
                .iter()
                .any(|c| c.starts_with("G10") || c.starts_with("$LocalFS/Run="))
        );
    }
    #[test]
    fn changing_controller_does_not_restore_another_machines_position() {
        let sim = Simulator::start().unwrap();
        let other = Simulator::start().unwrap();
        let checkpoint = CheckpointFile::new();
        let target = seed_position(&sim);
        let controller = checkpoint.start(&sim);
        restored(&controller, target);
        controller.send(Action::Connect {
            address: other.address.clone(),
            http: other.http.clone(),
        });
        wait(&controller, |s| {
            s.connected
                && s.initialized
                && s.status.position_known
                && s.status.wpos == Point::default()
        });
        assert!(!other.commands().iter().any(|c| c.starts_with("G10")));
        assert!(checkpoint.saved(&sim).unwrap().distance(target) < 1e-9);
        controller.send(Action::Connect {
            address: sim.address.clone(),
            http: sim.http.clone(),
        });
        restored(&controller, target);
    }
    #[test]
    fn full_upload_run_pause_restore_stop_and_reconnect() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized);
        controller.send(Action::Load {
            name: "test.nc".into(),
            text: "G21G90\nM3S1\nG4P0.123456\nG1X100F600\nM5\nM30".into(),
        });
        let ready = wait(&controller, |s| s.ready);
        let program = ready.program.unwrap();
        assert_eq!(program.total.dwell, 0.123456);
        assert_eq!(program.uploaded.matches("G4").count(), 1);
        assert!(protocol::verify(&sim.http, &program.uploaded).is_ok());
        controller.send(Action::Run);
        wait(&controller, |s| s.status.state == "Run" && s.status.torch);
        controller.send(Action::Pause);
        let held = wait(&controller, |s| s.phase == "Paused · torch off");
        assert!(!held.status.torch);
        let elapsed = held.elapsed;
        thread::sleep(Duration::from_millis(250));
        assert!((controller.snapshot().elapsed - elapsed).abs() < 0.05);
        controller.send(Action::Resume);
        let resumed = wait(&controller, |s| s.status.state == "Run");
        assert!(resumed.status.torch);
        sim.disconnect();
        wait(&controller, |s| !s.connected);
        let resumed = wait(&controller, |s| s.initialized);
        assert!(resumed.timing_uncertain);
        assert_eq!(
            sim.commands()
                .iter()
                .filter(|c| c.starts_with("$LocalFS/Run="))
                .count(),
            1,
            "Run must never be replayed"
        );
        controller.send(Action::Stop);
        let stopped = wait(&controller, |s| s.initialized && !s.job && s.status.idle());
        assert!(!stopped.status.torch);
    }
    #[test]
    fn jog_cancel_and_dro_do_not_move_machine_coordinates() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::Jog {
            step: 1.0,
            x: 1.0,
            y: 0.0,
            speed: 600.0,
        });
        wait(&controller, |s| s.status.state == "Jog");
        controller.send(Action::CancelJog);
        let before = wait(&controller, |s| s.status.idle());
        assert!(before.status.wpos.x > 0.0);
        controller.send(Action::SetCoordinate {
            axis: 'X',
            value: 25.0,
        });
        let after = wait(&controller, |s| {
            (s.status.wpos.x - 25.0).abs() < 0.001 && s.status.position_known
        });
        assert!((before.status.mpos.x - after.status.mpos.x).abs() < 0.001);
    }
    #[test]
    fn reconnect_while_paused_retains_torch_restore_intent() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::Load {
            name: "pause.nc".into(),
            text: "G21G90\nM3S1\nG1X100F600\nM5\nM30".into(),
        });
        wait(&controller, |s| s.can_run());
        controller.send(Action::Run);
        wait(&controller, |s| s.status.state == "Run" && s.status.torch);
        controller.send(Action::Pause);
        wait(&controller, |s| s.phase == "Paused · torch off");
        sim.disconnect();
        wait(&controller, |s| !s.connected);
        let s = wait(&controller, |s| {
            s.initialized && s.phase == "Paused · torch off"
        });
        assert!(!s.status.torch);
        controller.send(Action::Resume);
        let resumed = wait(&controller, |s| s.status.state == "Run");
        assert!(resumed.status.torch);
        controller.send(Action::Stop);
        wait(&controller, |s| s.initialized && !s.job && !s.status.torch);
    }
    #[test]
    fn completion_after_reconnect_does_not_wait_for_the_lost_run_ack() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::Load {
            name: "short.nc".into(),
            text: "G21G90\nM3S1\nG1X20F600\nM5\nM30".into(),
        });
        wait(&controller, |s| s.can_run());
        controller.send(Action::Run);
        wait(&controller, |s| s.status.state == "Run");
        sim.disconnect();
        wait(&controller, |s| !s.connected);
        wait(&controller, |s| s.initialized);
        let ended = wait(&controller, |s| !s.job && s.status.idle());
        assert!(ended.timing_uncertain);
        assert_eq!(
            sim.commands()
                .iter()
                .filter(|s| s.starts_with("$LocalFS/Run="))
                .count(),
            1
        );
    }
    #[test]
    fn verification_catches_same_length_corruption() {
        let sim = Simulator::start().unwrap();
        let data = "N1G0X1\n";
        protocol::upload(&sim.http, data, |_| {}).unwrap();
        sim.machine.lock().unwrap().file = "N1G0X9\n".into();
        assert!(protocol::verify(&sim.http, data).is_err());
    }
    #[test]
    fn completion_ack_handles_jobs_shorter_than_status_poll_interval() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::Load {
            name: "tiny.nc".into(),
            text: "G21G90\nG0X0\nM30".into(),
        });
        wait(&controller, |s| s.can_run());
        controller.send(Action::Run);
        let s = wait(&controller, |s| s.phase == "Completed");
        assert!(!s.job);
        assert_eq!(s.progress, 1.0);
    }
    #[test]
    fn hull_trace_is_closed_xy_only_torch_off_and_returns_to_start() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::SetCoordinate {
            axis: 'X',
            value: 3.0,
        });
        wait(&controller, |s| {
            (s.status.wpos.x - 3.0).abs() < 0.001 && s.status.position_known
        });
        controller.send(Action::SetCoordinate {
            axis: 'Y',
            value: 4.0,
        });
        wait(&controller, |s| {
            (s.status.wpos.y - 4.0).abs() < 0.001 && s.status.position_known
        });
        controller.send(Action::Load {
            name: "hull.nc".into(),
            text: "G21G90\nG0X10Y0\nM3S1\nG1X10Y10F600\nX5Y5\nX0Y10\nM5\nM30".into(),
        });
        wait(&controller, |s| s.can_run());
        let start = controller.snapshot().status.wpos;
        controller.send(Action::TraceHull { speed: 777.0 });
        let tracing = wait(&controller, |s| s.tracing);
        assert!(!tracing.can_run() && !tracing.can_jog());
        let done = wait(&controller, |s| s.phase == "Hull trace complete");
        assert!(!done.status.torch);
        assert!(done.status.wpos.distance(start) < 0.001);
        let commands = sim.commands();
        let jogs: Vec<_> = commands.iter().filter(|c| c.starts_with("$J=")).collect();
        assert_eq!(jogs.len(), 6); // four hull vertices, close, return
        for command in &jogs {
            assert!(command.starts_with("$J=G21G90X") && command.ends_with("F777.0"));
            assert!(!command.contains('Z') && !command.contains("M3"));
        }
        assert!(jogs.iter().any(|c| c.contains("X10.000Y10.000")));
        assert!(!commands.iter().any(|c| c.starts_with("$LocalFS/Run=")));
        controller.send(Action::TraceHull { speed: 600.0 });
        wait(&controller, |s| s.tracing && s.status.state == "Jog");
        controller.send(Action::CancelTrace);
        wait(&controller, |s| !s.tracing && s.status.idle());
        let count = sim.commands().len();
        thread::sleep(Duration::from_millis(350));
        assert_eq!(
            count,
            sim.commands().len(),
            "Trace must not resume after cancellation"
        );
    }
    #[test]
    fn console_is_separate_and_updates_dro_and_override_promptly() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        let started = Instant::now();
        controller.send(Action::Console("G10L20P1X42".into()));
        let changed = wait(&controller, |s| (s.status.wpos.x - 42.0).abs() < 0.001);
        assert!(started.elapsed() < Duration::from_millis(500));
        assert!(changed.console.iter().any(|l| l == "> G10L20P1X42"));
        assert!(changed.console.iter().any(|l| l == "ok"));
        assert!(
            !changed
                .log
                .iter()
                .any(|l| l.contains("G10L20P1X42") || l == "ok")
        );
        let started = Instant::now();
        controller.send(Action::Feed(0x91));
        wait(&controller, |s| {
            (s.status.feed_override - 1.1).abs() < 0.001
        });
        assert!(started.elapsed() < Duration::from_millis(500));
    }
    #[test]
    fn normal_stop_avoids_abort_alarm_and_unlock_clears_alarm_message() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::Load {
            name: "stop.nc".into(),
            text: "G21G90\nM3S1\nG1X100F600\nM5\nM30".into(),
        });
        wait(&controller, |s| s.can_run());
        controller.send(Action::Run);
        wait(&controller, |s| s.status.state == "Run" && s.status.torch);
        controller.send(Action::Console("$G".into()));
        wait(&controller, |s| {
            s.error
                .as_ref()
                .is_some_and(|e| e.contains("Ordinary console commands"))
        });
        assert!(
            controller.snapshot().job,
            "A rejected console command must not end a job"
        );
        controller.send(Action::Stop);
        wait(&controller, |s| s.initialized && !s.job && s.status.idle());
        assert!(sim.machine.lock().unwrap().alarm.is_none());
        assert!(!controller.snapshot().console.iter().any(|l| l == "ALARM:3"));
        // The simulator also models the old immediate-reset failure case.
        wait(&controller, |s| s.can_run());
        controller.send(Action::Run);
        wait(&controller, |s| s.status.state == "Run");
        controller.send(Action::Stop);
        controller.send(Action::Stop); // repeated Stop is an immediate reset fallback
        let alarm = wait(&controller, |s| {
            s.alarm.as_deref() == Some("ALARM:3") && !s.status.torch
        });
        assert!(!alarm.status.torch);
        wait(&controller, |s| s.status.state == "Alarm");
        controller.send(Action::Unlock);
        let unlocked = wait(&controller, |s| s.status.idle());
        assert!(unlocked.alarm.is_none());
    }
    #[test]
    fn hull_trace_rejects_lit_torch_and_never_restarts_after_reconnect() {
        let sim = Simulator::start().unwrap();
        let controller = Controller::start(sim.address.clone(), sim.http.clone(), None);
        wait(&controller, |s| s.initialized && s.status.position_known);
        controller.send(Action::Load {
            name: "trace.nc".into(),
            text: "G21G90\nG0X10\nG1Y10F600\nX0\nY0\nM30".into(),
        });
        wait(&controller, |s| s.can_run());
        sim.machine.lock().unwrap().torch = true;
        wait(&controller, |s| s.status.torch);
        controller.send(Action::TraceHull { speed: 600.0 });
        let rejected = wait(&controller, |s| {
            s.error.as_ref().is_some_and(|e| e.contains("torch off"))
        });
        assert!(!rejected.tracing);
        assert!(!sim.commands().iter().any(|c| c.starts_with("$J=")));
        sim.machine.lock().unwrap().torch = false;
        wait(&controller, |s| !s.status.torch);
        controller.send(Action::TraceHull { speed: 600.0 });
        wait(&controller, |s| s.tracing && s.status.state == "Jog");
        sim.disconnect();
        let disconnected = wait(&controller, |s| !s.connected);
        assert!(!disconnected.tracing);
        let count = sim
            .commands()
            .iter()
            .filter(|c| c.starts_with("$J="))
            .count();
        let reconnected = wait(&controller, |s| s.initialized && s.status.idle());
        assert!(!reconnected.tracing);
        assert_eq!(
            count,
            sim.commands()
                .iter()
                .filter(|c| c.starts_with("$J="))
                .count()
        );
    }
}
