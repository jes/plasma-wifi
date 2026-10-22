use crate::gcode::Point;
use anyhow::{Context, Result, bail, ensure};
use serde_json::Value;
use std::{
    io::{Read, Write},
    net::{TcpStream, ToSocketAddrs},
    time::{Duration, Instant},
};

pub const HOLD: u8 = b'!';
pub const RESUME: u8 = b'~';
pub const RESET: u8 = 0x18;
pub const JOG_CANCEL: u8 = 0x85;
pub const TORCH_TOGGLE: u8 = 0x9e;

#[derive(Clone, Debug)]
pub struct Status {
    pub state: String,
    pub mpos: Point,
    pub wpos: Point,
    pub wco: Point,
    pub position_known: bool,
    pub wco_known: bool,
    pub line: Option<usize>,
    pub feed_override: f64,
    pub rapid_override: f64,
    pub feed: f64,
    pub torch: bool,
    pub accessory_at: Option<Instant>,
    pub file: Option<String>,
    pub file_percent: Option<f64>,
    pub at: Option<Instant>,
}
impl Default for Status {
    fn default() -> Self {
        Self {
            state: "Disconnected".into(),
            mpos: Point::default(),
            wpos: Point::default(),
            wco: Point::default(),
            position_known: false,
            wco_known: false,
            line: None,
            feed_override: 1.0,
            rapid_override: 1.0,
            feed: 0.0,
            torch: false,
            accessory_at: None,
            file: None,
            file_percent: None,
            at: None,
        }
    }
}
impl Status {
    pub fn idle(&self) -> bool {
        self.state == "Idle" && self.file.is_none()
    }
    pub fn held(&self) -> bool {
        self.state == "Hold:0"
    }
    pub fn fresh(&self) -> bool {
        self.at
            .is_some_and(|t| t.elapsed() < Duration::from_secs(2))
    }
    pub fn update(&mut self, line: &str, inches: bool) -> bool {
        let Some(body) = line.strip_prefix('<').and_then(|s| s.strip_suffix('>')) else {
            return false;
        };
        let mut parts = body.split('|');
        self.state = parts.next().unwrap_or("Unknown").into();
        self.line = None;
        self.file = None;
        self.file_percent = None;
        let scale = if inches { 25.4 } else { 1.0 };
        let mut mpos = None;
        let mut wpos = None;
        for part in parts {
            if let Some(v) = part.strip_prefix("MPos:") {
                mpos = point(v, scale);
            }
            if let Some(v) = part.strip_prefix("WPos:") {
                wpos = point(v, scale);
            }
            if let Some(v) = part.strip_prefix("WCO:").and_then(|v| point(v, scale)) {
                self.wco = v;
                self.wco_known = true;
            }
            if let Some(v) = part.strip_prefix("Ln:") {
                self.line = v.parse().ok();
            }
            if let Some(v) = part.strip_prefix("FS:") {
                self.feed = v
                    .split(',')
                    .next()
                    .and_then(|v| v.parse::<f64>().ok())
                    .unwrap_or(0.0)
                    * scale;
            }
            if let Some(v) = part.strip_prefix("Ov:") {
                let mut values = v.split(',').filter_map(|v| v.parse::<f64>().ok());
                self.feed_override = values.next().unwrap_or(100.0) / 100.0;
                self.rapid_override = values.next().unwrap_or(100.0) / 100.0;
                // FluidNC omits A entirely when all accessories are off, but only
                // refreshes it on Ov reports. Absence on other reports means unchanged.
                self.torch = false;
                self.accessory_at = Some(Instant::now());
            }
            if let Some(v) = part.strip_prefix("A:") {
                self.torch = v.contains('S') || v.contains('C');
                self.accessory_at = Some(Instant::now());
            }
            if let Some((percent, file)) = part.strip_prefix("SD:").and_then(|v| v.split_once(','))
            {
                self.file_percent = percent.parse().ok();
                self.file = Some(file.into());
            }
        }
        if let Some(v) = mpos {
            self.mpos = v;
            if self.wco_known {
                self.wpos = Point {
                    x: v.x - self.wco.x,
                    y: v.y - self.wco.y,
                    z: v.z - self.wco.z,
                };
                self.position_known = true;
            }
        }
        if let Some(v) = wpos {
            self.wpos = v;
            self.position_known = true;
            if self.wco_known {
                self.mpos = Point {
                    x: v.x + self.wco.x,
                    y: v.y + self.wco.y,
                    z: v.z + self.wco.z,
                };
            }
        }
        self.at = Some(Instant::now());
        true
    }
}
fn point(s: &str, scale: f64) -> Option<Point> {
    let mut v = s
        .split(',')
        .map(|v| v.parse::<f64>().ok().filter(|n| n.is_finite()));
    Some(Point {
        x: v.next()?? * scale,
        y: v.next()?? * scale,
        z: v.next().flatten().unwrap_or(0.0) * scale,
    })
}

/// Telnet framing is handled below the FluidNC line protocol. Realtime bytes
/// (including high-bit overrides) are sent unmodified, as required by FluidNC.
pub struct Link {
    stream: TcpStream,
    line: Vec<u8>,
    telnet: u8,
    negotiation: u8,
}
impl Link {
    pub fn connect(address: &str) -> Result<Self> {
        let address = address
            .to_socket_addrs()?
            .next()
            .context("Address has no IP")?;
        let stream = TcpStream::connect_timeout(&address, Duration::from_millis(800))?;
        stream.set_nonblocking(true)?;
        stream.set_nodelay(true)?;
        Ok(Self {
            stream,
            line: Vec::new(),
            telnet: 0,
            negotiation: 0,
        })
    }
    pub fn send(&mut self, bytes: &[u8]) -> Result<()> {
        self.stream
            .write_all(bytes)
            .context("Controller write failed")
    }
    pub fn command(&mut self, command: &str) -> Result<()> {
        self.send(format!("{command}\n").as_bytes())
    }
    pub fn poll(&mut self) -> Result<Vec<String>> {
        let mut lines = Vec::new();
        let mut buffer = [0u8; 4096];
        loop {
            let count = match self.stream.read(&mut buffer) {
                Ok(0) => bail!("Controller closed the connection"),
                Ok(n) => n,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e.into()),
            };
            for &b in &buffer[..count] {
                match self.telnet {
                    1 => match b {
                        255 => {
                            self.telnet = 0;
                        }
                        251..=254 => {
                            self.negotiation = b;
                            self.telnet = 2;
                            continue;
                        }
                        250 => {
                            self.telnet = 3;
                            continue;
                        }
                        _ => {
                            self.telnet = 0;
                            continue;
                        }
                    },
                    2 => {
                        let reply = match self.negotiation {
                            251 | 252 => 254,
                            _ => 252,
                        };
                        self.send(&[255, reply, b])?;
                        self.telnet = 0;
                        continue;
                    }
                    3 => {
                        if b == 255 {
                            self.telnet = 4;
                        }
                        continue;
                    }
                    4 => {
                        self.telnet = if b == 240 { 0 } else { 3 };
                        continue;
                    }
                    _ if b == 255 => {
                        self.telnet = 1;
                        continue;
                    }
                    _ => {}
                }
                if b == b'\n' {
                    let line = String::from_utf8_lossy(&self.line).trim().to_owned();
                    if !line.is_empty() {
                        lines.push(line);
                    }
                    self.line.clear();
                } else if b != b'\r' {
                    self.line.push(b);
                    ensure!(self.line.len() < 65536, "Oversized controller response");
                }
            }
        }
        Ok(lines)
    }
}

#[derive(Clone, Debug)]
pub struct FileInfo {
    pub bytes: usize,
    pub free: Option<String>,
}

fn agent() -> ureq::Agent {
    ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(2))
        .timeout_read(Duration::from_secs(20))
        .timeout_write(Duration::from_secs(20))
        .build()
}

pub fn file_list(base: &str) -> Result<Value> {
    Ok(agent()
        .get(&format!("{base}/files?path=/"))
        .call()
        .context("Read LocalFS listing")?
        .into_json()?)
}
fn file_size(list: &Value) -> Option<usize> {
    let file = list.get("files")?.as_array()?.iter().find(|f| {
        f["name"]
            .as_str()
            .is_some_and(|n| n.trim_start_matches('/') == "plasma.nc")
    })?;
    file["size"]
        .as_u64()
        .map(|n| n as usize)
        .or_else(|| file["size"].as_str()?.parse().ok())
}
fn free_space(list: &Value) -> Option<String> {
    // The firmware returns human-readable total/used strings, so keep their precision explicit.
    let used = list.get("used")?.as_str()?;
    let total = list.get("total")?.as_str()?;
    let bytes = |s: &str| -> Option<f64> {
        let n = s.find(|c: char| c.is_alphabetic()).unwrap_or(s.len());
        let value: f64 = s[..n].trim().parse().ok()?;
        let unit = s[n..].trim().to_ascii_uppercase();
        Some(
            value
                * match unit.as_str() {
                    "KB" => 1024.0,
                    "MB" => 1024.0 * 1024.0,
                    "GB" => 1024.0 * 1024.0 * 1024.0,
                    _ => 1.0,
                },
        )
    };
    Some(format!(
        "≈ {:.0} bytes free ({used} / {total})",
        (bytes(total)? - bytes(used)?).max(0.0)
    ))
}
pub fn verify(base: &str, data: &str) -> Result<FileInfo> {
    let listing = file_list(base)?;
    let bytes = file_size(&listing).context("plasma.nc missing from LocalFS")?;
    ensure!(
        bytes == data.len(),
        "Upload size mismatch: expected {}, found {bytes}",
        data.len()
    );
    let mut downloaded = Vec::new();
    agent()
        .get(&format!("{base}/localfs/plasma.nc"))
        .call()
        .context("Download for verification")?
        .into_reader()
        .take(data.len() as u64 + 1)
        .read_to_end(&mut downloaded)?;
    ensure!(
        downloaded == data.as_bytes(),
        "Upload contents differ from the local processed file"
    );
    Ok(FileInfo {
        bytes,
        free: free_space(&listing),
    })
}
pub fn upload(base: &str, data: &str, mut progress: impl FnMut(&str)) -> Result<FileInfo> {
    progress("Checking LocalFS");
    let listing = file_list(base)?;
    if file_size(&listing).is_some() {
        progress("Replacing plasma.nc");
        let result: Value = agent()
            .get(&format!(
                "{base}/files?path=/&action=delete&filename=plasma.nc"
            ))
            .call()?
            .into_json()?;
        ensure!(
            !result["status"]
                .as_str()
                .unwrap_or("")
                .starts_with("Cannot"),
            "Delete failed: {}",
            result["status"]
        );
    }
    progress("Uploading plasma.nc");
    let boundary = "PlasmaWiFiBoundary7f6c3d";
    let body = format!(
        "--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"plasma.nc\"\r\nContent-Type: application/octet-stream\r\n\r\n{data}\r\n--{boundary}--\r\n"
    );
    let response: Value = agent()
        .post(&format!("{base}/files?path=/&plasma.ncS={}", data.len()))
        .set(
            "Content-Type",
            &format!("multipart/form-data; boundary={boundary}"),
        )
        .send_bytes(body.as_bytes())?
        .into_json()?;
    ensure!(
        response["status"].as_str().is_none_or(|s| s == "Ok"),
        "Upload failed: {}",
        response["status"]
    );
    progress("Verifying size and contents");
    verify(base, data)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn status_tracks_sparse_accessories_and_coordinates() {
        let mut s = Status::default();
        assert!(s.update(
            "<Run|MPos:10,20,0|WCO:2,3,0|Ln:12|Ov:150,100,100|A:S|SD:10.00,/localfs/plasma.nc>",
            false
        ));
        assert_eq!(s.wpos.x, 8.0);
        assert!(s.torch);
        assert_eq!(s.line, Some(12));
        s.update("<Hold:0|MPos:11,20,0>", false);
        assert!(s.torch);
        assert!(s.held());
        s.update("<Hold:0|MPos:11,20,0|Ov:150,100,100>", false);
        assert!(!s.torch);
        s.update("<Idle|WPos:1,2,0|WCO:0,0,0>", true);
        assert_eq!(s.wpos.x, 25.4);
    }
}
