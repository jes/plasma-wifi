use anyhow::{Context, Result, bail, ensure};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl Point {
    pub fn distance(self, b: Self) -> f64 {
        ((self.x - b.x).powi(2) + (self.y - b.y).powi(2) + (self.z - b.z).powi(2)).sqrt()
    }
    pub fn lerp(self, b: Self, t: f64) -> Self {
        Self {
            x: self.x + (b.x - self.x) * t,
            y: self.y + (b.y - self.y) * t,
            z: self.z + (b.z - self.z) * t,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TimeParts {
    pub feed: f64,
    pub rapid: f64,
    pub dwell: f64,
}
impl TimeParts {
    pub fn seconds(self, feed_override: f64, rapid_override: f64, coefficient: f64) -> f64 {
        (self.feed / feed_override.max(0.01) + self.rapid / rapid_override.max(0.01)) * coefficient
            + self.dwell
    }
    pub fn plus(self, b: Self) -> Self {
        Self {
            feed: self.feed + b.feed,
            rapid: self.rapid + b.rapid,
            dwell: self.dwell + b.dwell,
        }
    }
    pub fn minus(self, b: Self) -> Self {
        Self {
            feed: (self.feed - b.feed).max(0.0),
            rapid: (self.rapid - b.rapid).max(0.0),
            dwell: (self.dwell - b.dwell).max(0.0),
        }
    }
    pub fn scaled(self, t: f64) -> Self {
        Self {
            feed: self.feed * t,
            rapid: self.rapid * t,
            dwell: self.dwell * t,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Block {
    pub n: usize,
    pub source: usize,
    pub code: String,
    pub path: Vec<Point>,
    pub rapid: bool,
    pub torch: bool,
    pub time: TimeParts,
    pub before: TimeParts,
}

impl Block {
    /// Project onto the sampled path, including helical Z distance.
    pub fn fraction_after(&self, position: Point, minimum: f64) -> f64 {
        let total: f64 = self.path.windows(2).map(|p| p[0].distance(p[1])).sum();
        if total < 1e-9 {
            return 0.0;
        }
        let mut best = (f64::INFINITY, 0.0);
        let mut along = 0.0;
        for p in self.path.windows(2) {
            let length = p[0].distance(p[1]);
            let d = Point {
                x: p[1].x - p[0].x,
                y: p[1].y - p[0].y,
                z: p[1].z - p[0].z,
            };
            let t = if length < 1e-9 {
                0.0
            } else {
                (((position.x - p[0].x) * d.x
                    + (position.y - p[0].y) * d.y
                    + (position.z - p[0].z) * d.z)
                    / length.powi(2))
                .clamp(0.0, 1.0)
            };
            let error = position.distance(p[0].lerp(p[1], t));
            let fraction = (along + length * t) / total;
            if error + 1e-5 < best.0 && fraction + 1e-5 >= minimum {
                best = (error, fraction.max(minimum));
            }
            along += length;
        }
        if best.0.is_finite() { best.1 } else { minimum }
    }
}

#[derive(Clone, Debug)]
pub struct Program {
    pub original: String,
    pub source_lines: Vec<String>,
    pub uploaded: String,
    pub blocks: Vec<Block>,
    pub total: TimeParts,
    pub warnings: Vec<String>,
    pub max_source_columns: usize,
    pub max_uploaded_columns: usize,
}

impl Program {
    pub fn hull(&self) -> Vec<Point> {
        let mut points: Vec<_> = self
            .blocks
            .iter()
            .flat_map(|b| b.path.iter())
            .map(|p| Point {
                x: p.x,
                y: p.y,
                z: 0.0,
            })
            .collect();
        points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
        points.dedup_by(|a, b| a.x == b.x && a.y == b.y);
        if points.len() < 3 {
            return points;
        }
        let cross =
            |a: Point, b: Point, c: Point| (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        let mut lower = Vec::new();
        for &point in &points {
            while lower.len() >= 2
                && cross(lower[lower.len() - 2], lower[lower.len() - 1], point) <= 0.0
            {
                lower.pop();
            }
            lower.push(point);
        }
        let mut upper = Vec::new();
        for &point in points.iter().rev() {
            while upper.len() >= 2
                && cross(upper[upper.len() - 2], upper[upper.len() - 1], point) <= 0.0
            {
                upper.pop();
            }
            upper.push(point);
        }
        lower.pop();
        upper.pop();
        lower.extend(upper);
        lower
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ParseOptions {
    pub start: Point,
    pub wco: Point,
    pub rapid: Point,
    pub initial_feed: f64,
    pub initial_mm: bool,
    pub initial_absolute: bool,
    pub initial_motion: u8,
    pub initial_spindle: bool,
    pub initial_speed: f64,
    pub initial_xy_plane: bool,
    pub initial_feed_per_minute: bool,
}
impl Default for ParseOptions {
    fn default() -> Self {
        Self {
            start: Point::default(),
            wco: Point::default(),
            rapid: Point {
                x: 3000.0,
                y: 3000.0,
                z: 1000.0,
            },
            initial_feed: 0.0,
            initial_mm: true,
            initial_absolute: true,
            initial_motion: 0,
            initial_spindle: false,
            initial_speed: 0.0,
            initial_xy_plane: true,
            initial_feed_per_minute: true,
        }
    }
}

#[derive(Clone, Debug)]
struct Word {
    letter: char,
    value: f64,
    text: String,
}

/// Compact words before analysis so the preview and timing match the uploaded coordinates.
fn words(line: &str) -> Result<Vec<Word>> {
    let mut clean = String::new();
    let mut depth = 0;
    for c in line.chars() {
        match c {
            ';' if depth == 0 => break,
            '(' => {
                ensure!(
                    depth == 0,
                    "nested parenthesized comments are ambiguous in FluidNC"
                );
                depth = 1;
            }
            ')' => {
                ensure!(depth > 0, "unmatched closing comment");
                depth -= 1;
            }
            _ if depth > 0 => {}
            _ if c.is_ascii_whitespace() => {}
            _ => {
                ensure!(c.is_ascii(), "non-ASCII G-code outside comments");
                clean.push(c.to_ascii_uppercase());
            }
        }
    }
    ensure!(depth == 0, "unclosed parenthesized comment");
    ensure!(
        clean != "%",
        "percent delimiters must be on a line by themselves"
    );
    if clean.is_empty() {
        return Ok(Vec::new());
    }
    let chars: Vec<char> = clean.chars().collect();
    let mut i = 0;
    let mut out = Vec::new();
    while i < chars.len() {
        let letter = chars[i];
        ensure!(
            letter.is_ascii_alphabetic(),
            "unsupported syntax near '{}'; expressions, checksums and macros are not supported",
            &clean[i..]
        );
        i += 1;
        let start = i;
        if i < chars.len() && (chars[i] == '+' || chars[i] == '-') {
            i += 1;
        }
        while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
            i += 1;
        }
        ensure!(i > start, "missing number after {letter}");
        let number: String = chars[start..i].iter().collect();
        let mut value: f64 = number
            .parse()
            .with_context(|| format!("invalid number {number}"))?;
        ensure!(value.is_finite(), "non-finite number");
        if letter != 'N' {
            let text = if matches!(letter, 'X' | 'Y' | 'Z' | 'I' | 'J' | 'K' | 'R') {
                let rounded = format!("{value:.3}");
                value = rounded.parse()?;
                let compact = rounded.trim_end_matches('0').trim_end_matches('.');
                // Only strip trailing zeroes after the decimal point, never integer zeroes.
                let compact = if value == 0.0 { "0" } else { compact };
                format!("{letter}{compact}")
            } else if letter == 'G' {
                format!("G{value}")
            } else {
                format!("{letter}{number}")
            };
            out.push(Word {
                letter,
                value,
                text,
            });
        }
    }
    Ok(out)
}

struct Parser {
    opts: ParseOptions,
    position: Point,
    unit: f64,
    absolute: bool,
    motion: u8,
    feed: f64,
    speed: f64,
    spindle: bool,
    total: TimeParts,
    blocks: Vec<Block>,
    warnings: Vec<String>,
    ended: bool,
    xy_plane: bool,
    feed_per_minute: bool,
}

pub fn parse(original: &str, opts: ParseOptions) -> Result<Program> {
    let mut p = Parser {
        opts,
        position: opts.start,
        unit: if opts.initial_mm { 1.0 } else { 25.4 },
        absolute: opts.initial_absolute,
        motion: opts.initial_motion,
        feed: opts.initial_feed,
        speed: opts.initial_speed,
        spindle: opts.initial_spindle,
        total: TimeParts::default(),
        blocks: Vec::new(),
        warnings: Vec::new(),
        ended: false,
        xy_plane: opts.initial_xy_plane,
        feed_per_minute: opts.initial_feed_per_minute,
    };
    let source_lines: Vec<String> = original.lines().map(str::to_owned).collect();
    let mut saw_nonblank = false;
    for (source, line) in source_lines.iter().enumerate() {
        if line.trim() == "%" {
            if saw_nonblank {
                p.warnings.push(format!("File ends at the % delimiter on source line {}; following text is not uploaded",source+1));
                break;
            }
            saw_nonblank = true;
            continue;
        }
        if !line.trim().is_empty() {
            saw_nonblank = true;
        }
        let w = words(line).with_context(|| format!("Line {}", source + 1))?;
        if w.is_empty() {
            continue;
        }
        if p.ended {
            p.warnings.push(format!(
                "Line {} follows M2/M30 and will not execute",
                source + 1
            ));
        }
        p.process(&w, source)
            .with_context(|| format!("Line {}", source + 1))?;
    }
    ensure!(
        !p.blocks.is_empty(),
        "The file contains no executable G-code"
    );
    let uploaded = p
        .blocks
        .iter()
        .map(|b| format!("N{}{}\n", b.n, b.code))
        .collect();
    Ok(Program {
        max_source_columns: source_lines
            .iter()
            .map(|s| s.chars().count() + 8)
            .max()
            .unwrap_or(0),
        max_uploaded_columns: p.blocks.iter().map(|b| b.code.len() + 8).max().unwrap_or(0),
        original: original.to_owned(),
        source_lines,
        uploaded,
        blocks: p.blocks,
        total: p.total,
        warnings: p.warnings,
    })
}

impl Parser {
    fn process(&mut self, w: &[Word], source: usize) -> Result<()> {
        validate_modal_groups(w)?;
        let get = |letter| w.iter().find(|w| w.letter == letter).map(|w| w.value);
        for letter in "XYZIJKRFPST".chars() {
            ensure!(
                w.iter().filter(|w| w.letter == letter).count() <= 1,
                "duplicate {letter} word"
            );
        }
        let mut dwell = false;
        let mut machine = false;
        let mut no_motion = false;
        let mut end = false;
        for word in w {
            match word.letter {
                'G' => match word.value {
                    0.0 => self.motion = 0,
                    1.0 => self.motion = 1,
                    2.0 => self.motion = 2,
                    3.0 => self.motion = 3,
                    4.0 => {
                        dwell = true;
                        no_motion = true;
                    }
                    17.0 => self.xy_plane = true,
                    20.0 => self.unit = 25.4,
                    21.0 => self.unit = 1.0,
                    90.0 => self.absolute = true,
                    91.0 => self.absolute = false,
                    94.0 => self.feed_per_minute = true,
                    91.1 | 40.0 | 49.0 | 54.0 | 61.0 => {}
                    53.0 => machine = true,
                    80.0 => {
                        self.motion = 80;
                        no_motion = true;
                    }
                    _ => bail!(
                        "G{} is not supported by the XY preview/processor; revise the CAM post rather than approximating it",
                        word.value
                    ),
                },
                'M' => match word.value {
                    3.0 => self.spindle = true,
                    5.0 => self.spindle = false,
                    2.0 | 30.0 => end = true,
                    0.0 | 1.0 => self.warnings.push(format!(
                        "Line {} includes a program pause; use Pause/Resume to control the torch",
                        source + 1
                    )),
                    6.0 | 7.0 | 8.0 | 9.0 => {}
                    _ => bail!(
                        "M{} is not supported (torch control uses M3/M5)",
                        word.value
                    ),
                },
                'X' | 'Y' | 'Z' | 'I' | 'J' | 'K' | 'R' | 'F' | 'P' | 'S' | 'T' => {}
                _ => bail!("{} words are not supported", word.letter),
            }
        }
        if let Some(f) = get('F') {
            ensure!(f > 0.0, "feed must be positive");
            self.feed = f * self.unit;
        }
        if let Some(s) = get('S') {
            ensure!(s >= 0.0, "spindle speed must be nonnegative");
            self.speed = s;
        }
        let start = self.position;
        let mut target = start;
        for (letter, value, offset) in [
            ('X', &mut target.x, self.opts.wco.x),
            ('Y', &mut target.y, self.opts.wco.y),
            ('Z', &mut target.z, self.opts.wco.z),
        ] {
            if let Some(v) = get(letter) {
                *value = if machine {
                    v * self.unit - offset
                } else if self.absolute {
                    v * self.unit
                } else {
                    *value + v * self.unit
                };
            }
        }
        if get('Z').is_some() && !self.warnings.iter().any(|s| s.starts_with("Z moves")) {
            self.warnings.push("Z moves are preserved in the controller file; this machine has no physical Z axis. Configure CAM for plasma/XY cutting.".into());
        }
        let mut time = TimeParts::default();
        let mut path = Vec::new();
        let mut rapid = false;
        let movement = !no_motion
            && (get('X').is_some()
                || get('Y').is_some()
                || get('Z').is_some()
                || matches!(self.motion, 2 | 3) && (get('I').is_some() || get('J').is_some()));
        ensure!(
            matches!(self.motion, 2 | 3)
                || get('I').is_none()
                    && get('J').is_none()
                    && get('K').is_none()
                    && get('R').is_none(),
            "I/J/K/R words require an arc motion"
        );
        if dwell {
            time.dwell = get('P').context("G4 requires a P duration in seconds")?;
            ensure!(time.dwell >= 0.0, "dwell must be nonnegative");
        }
        if movement {
            ensure!(self.motion <= 3, "axis words without an active motion mode");
            ensure!(
                !machine || self.absolute && self.motion <= 1,
                "G53 requires absolute G0/G1"
            );
            let length;
            if matches!(self.motion, 2 | 3) {
                ensure!(
                    self.xy_plane,
                    "Arcs require the XY plane; include G17 in the file"
                );
                ensure!(
                    get('X').is_some() || get('Y').is_some(),
                    "FluidNC arcs require an X or Y word, including full circles"
                );
                ensure!(
                    get('R').is_none()
                        || get('I').is_none() && get('J').is_none() && get('K').is_none(),
                    "arc cannot combine R and I/J"
                );
                let center = if let Some(r) = get('R') {
                    radius_center(start, target, r * self.unit, self.motion == 2)?
                } else {
                    ensure!(
                        get('I').is_some() || get('J').is_some(),
                        "arc requires I/J or R"
                    );
                    Point {
                        x: start.x + get('I').unwrap_or(0.0) * self.unit,
                        y: start.y + get('J').unwrap_or(0.0) * self.unit,
                        z: start.z,
                    }
                };
                let r = (start.x - center.x).hypot(start.y - center.y);
                let r_end = (target.x - center.x).hypot(target.y - center.y);
                ensure!(
                    r > 1e-9 && (r - r_end).abs() <= 0.005f64.max(r * 0.001),
                    "arc endpoints disagree with center/radius"
                );
                let mut angle = sweep(start, target, center, self.motion == 2);
                if let Some(turns) = get('P') {
                    ensure!(
                        turns >= 0.0 && turns.fract() == 0.0 && turns <= 1000.0,
                        "arc P must be an integer from 0 to 1000"
                    );
                    angle += angle.signum() * (turns - 1.0).max(0.0) * TAU;
                }
                length = (r * angle.abs()).hypot(target.z - start.z);
                let segments = ((angle.abs() / (0.05f64.min((0.1 / r).sqrt() * 2.0))).ceil()
                    as usize)
                    .clamp(2, 20000);
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let mut fractions: Vec<f64> = (0..=segments)
                    .map(|step| step as f64 / segments as f64)
                    .collect();
                // Include exact XY extrema so a hull trace reaches the full axis extents,
                // even when an arc's ordinary tessellation misses a cardinal angle.
                let end_angle = a0 + angle;
                let quarter = std::f64::consts::FRAC_PI_2;
                for k in (a0.min(end_angle) / quarter).ceil() as i64
                    ..=(a0.max(end_angle) / quarter).floor() as i64
                {
                    fractions.push(((k as f64 * quarter - a0) / angle).clamp(0.0, 1.0));
                }
                fractions.sort_by(f64::total_cmp);
                fractions.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
                for t in fractions {
                    path.push(Point {
                        x: center.x + r * (a0 + angle * t).cos(),
                        y: center.y + r * (a0 + angle * t).sin(),
                        z: start.z + (target.z - start.z) * t,
                    });
                }
                *path.last_mut().unwrap() = target;
            } else {
                length = start.distance(target);
                path = vec![start, target];
            }
            if self.motion == 0 {
                rapid = true;
                // Axis limits constrain the component speeds of a diagonal rapid.
                time.rapid = ((target.x - start.x).abs() / self.opts.rapid.x.max(1.0))
                    .max((target.y - start.y).abs() / self.opts.rapid.y.max(1.0))
                    .max((target.z - start.z).abs() / self.opts.rapid.z.max(1.0))
                    * 60.0;
            } else {
                ensure!(
                    self.feed_per_minute,
                    "Feed motion requires units-per-minute mode; include G94 in the file"
                );
                ensure!(
                    self.feed > 0.0 || length < 1e-9,
                    "cutting move has no feed rate; include F in the file"
                );
                time.feed = if length < 1e-9 {
                    0.0
                } else {
                    length / self.feed * 60.0
                };
            }
            self.position = target;
        }
        // M2/M30 stop the torch, but after any motion on the same block.
        let code: String = w.iter().map(|w| w.text.as_str()).collect();
        ensure!(
            code.len() + 16 < 240,
            "block is too long for the controller input buffer"
        );
        let block_time = if self.ended {
            TimeParts::default()
        } else {
            time
        };
        self.blocks.push(Block {
            n: self.blocks.len() + 1,
            source,
            code,
            path: if self.ended { Vec::new() } else { path },
            rapid,
            torch: self.spindle && self.speed > 0.0,
            time: block_time,
            before: self.total,
        });
        self.total = self.total.plus(block_time);
        if end {
            self.spindle = false;
            self.ended = true;
        }
        Ok(())
    }
}

fn validate_modal_groups(words: &[Word]) -> Result<()> {
    let mut groups = std::collections::HashSet::new();
    for word in words {
        let group = match (word.letter, word.value) {
            ('G', 0.0 | 1.0 | 2.0 | 3.0 | 80.0) => Some(1),
            ('G', 4.0 | 53.0) => Some(2),
            ('G', 17.0 | 18.0 | 19.0) => Some(3),
            ('G', 20.0 | 21.0) => Some(4),
            ('G', 90.0 | 91.0) => Some(5),
            ('G', 54.0..=59.3) => Some(6),
            ('M', 3.0 | 4.0 | 5.0) => Some(7),
            ('M', 0.0 | 1.0 | 2.0 | 30.0) => Some(8),
            ('M', 7.0 | 8.0 | 9.0) => Some(9),
            _ => None,
        };
        if let Some(group) = group {
            ensure!(
                groups.insert(group),
                "multiple commands from the same modal group"
            );
        }
    }
    Ok(())
}

fn sweep(start: Point, end: Point, center: Point, clockwise: bool) -> f64 {
    let a = (start.y - center.y).atan2(start.x - center.x);
    let b = (end.y - center.y).atan2(end.x - center.x);
    let mut delta = (b - a).rem_euclid(TAU);
    if delta < 1e-10 {
        delta = TAU;
    }
    if clockwise {
        if delta == TAU { -TAU } else { delta - TAU }
    } else {
        delta
    }
}

fn radius_center(start: Point, end: Point, radius: f64, clockwise: bool) -> Result<Point> {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let chord = dx.hypot(dy);
    ensure!(chord > 1e-9, "full-circle arcs require I/J, not R");
    ensure!(
        chord <= 2.0 * radius.abs() + 1e-7,
        "arc radius is smaller than half the chord"
    );
    let h = (radius.powi(2) - (chord / 2.0).powi(2)).max(0.0).sqrt();
    let sign = if clockwise { -1.0 } else { 1.0 } * if radius < 0.0 { -1.0 } else { 1.0 };
    Ok(Point {
        x: (start.x + end.x) / 2.0 - sign * h * dy / chord,
        y: (start.y + end.y) / 2.0 + sign * h * dx / chord,
        z: start.z,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn program(s: &str) -> Program {
        parse(s, ParseOptions::default()).unwrap()
    }
    #[test]
    fn rounds_uploaded_coordinates_and_maps_source_lines() {
        let p = program("(FreeCAD)\nN10 G21 G90\n M3 S1 ; torch\nG1 X1.234567890 F60\nM5\n");
        assert_eq!(p.blocks[1].code, "M3S1");
        assert_eq!(p.blocks[1].source, 2);
        assert_eq!(p.blocks[2].source, 3);
        assert!(p.uploaded.contains("N3G1X1.235F60\n"));
        assert!((p.total.feed - 1.235).abs() < 1e-9);
        assert_eq!(p.total.dwell, 0.0);
    }
    #[test]
    fn arcs_units_incremental_and_helices() {
        let p = program("G21G90\nG0X10\nG3X0Y10I-10J0F600\nG91\nG1X1\nG20\nG1X1F60");
        assert!((p.blocks[2].time.feed - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
        assert!((p.blocks.last().unwrap().path.last().unwrap().x - 26.4).abs() < 1e-9);
        let p = program("G0X10\nG2X10Y0I-10J0F600");
        assert!((p.total.feed - TAU).abs() < 1e-8);
    }
    #[test]
    fn radius_sign_selects_major_arc() {
        let a = program("G2X10Y0R10F60");
        let b = program("G2X10Y0R-10F60");
        assert!(a.total.feed < b.total.feed);
        assert!((a.total.feed + b.total.feed - TAU * 10.0).abs() < 1e-8);
    }
    #[test]
    fn combined_ignition_and_motion_stay_in_one_block() {
        let p = program("G1X10M3S1F60");
        assert_eq!(p.uploaded, "N1G1X10M3S1F60\n");
        assert_eq!(p.blocks[0].time.feed, 10.0);
        assert!(p.blocks[0].torch);
        assert_eq!(p.total.dwell, 0.0);
    }
    #[test]
    fn cam_dwells_are_preserved_and_timed_without_extra_delays() {
        let p = program("M3S1\nG4P0.123456\nG1X10F60\nM5\nM3S1G4P1.75\nG1X20\nM5");
        assert_eq!(
            p.uploaded,
            "N1M3S1\nN2G4P0.123456\nN3G1X10F60\nN4M5\nN5M3S1G4P1.75\nN6G1X20\nN7M5\n"
        );
        assert_eq!(p.blocks[4].source, 4);
        assert_eq!(p.total.feed, 20.0);
        assert_eq!(p.total.dwell, 0.123456 + 1.75);
        assert!((p.total.seconds(2.0, 1.0, 1.0) - (10.0 + 0.123456 + 1.75)).abs() < 1e-9);
    }
    #[test]
    fn rejects_ambiguous_input_instead_of_rewriting() {
        for s in [
            "G1X#1F60",
            "G18G2X2I1F60",
            "G1X1",
            "G2X10R1F60",
            "G1X1(comment",
            "G1X1X2F60",
            "G93G1X1F60",
        ] {
            assert!(parse(s, ParseOptions::default()).is_err(), "{s}");
        }
    }
    #[test]
    fn override_never_scales_dwell_or_rapid_as_feed() {
        let t = TimeParts {
            feed: 100.0,
            rapid: 20.0,
            dwell: 5.0,
        };
        assert_eq!(t.seconds(2.0, 1.0, 1.0), 75.0);
    }
    #[test]
    fn freecad_grbl_preamble_and_machine_retract() {
        let p =
            program("G17G54G40G49G80G90G21\nT1M6\nS1M3\nG0X0Y0Z5\nG1X20F100\nM5\nG53G0X0Y0\nM30");
        assert_eq!(p.total.feed, 12.0);
        assert!(p.warnings.iter().any(|w| w.starts_with("Z moves")));
    }
    #[test]
    fn percent_end_marker_does_not_enable_unreachable_motion() {
        let p = program("%\nG21G90\nG1X10F60\n%\nG1X999");
        assert_eq!(p.total.feed, 10.0);
        assert!(!p.uploaded.contains("999"));
        assert!(parse("(comment)\n%\nG1X999F60", ParseOptions::default()).is_err());
    }
    #[test]
    fn speed_only_ignition_and_zero_speed_preserve_blocks() {
        let p = program("M3S0\nS1G1X10F60");
        assert_eq!(p.uploaded, "N1M3S0\nN2S1G1X10F60\n");
        assert!(p.blocks[1].torch);
        assert_eq!(p.total.dwell, 0.0);
        let p = program("G1X10F60M3S0");
        assert!(!p.blocks[0].torch);
        assert_eq!(p.total.dwell, 0.0);
    }
    #[test]
    fn modal_duplicates_are_rejected() {
        assert!(parse("G0G1X10M3S1F60", ParseOptions::default()).is_err());
        assert!(parse("M3M5S1G1X10F60", ParseOptions::default()).is_err());
    }
    #[test]
    fn initial_motion_and_arc_rotations_are_modelled() {
        let p = parse(
            "X10",
            ParseOptions {
                initial_motion: 1,
                initial_feed: 60.0,
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(p.total.feed, 10.0);
        let p = program("G0X10\nG3X10Y0I-10J0P2F600");
        assert!((p.total.feed - TAU * 2.0).abs() < 1e-8);
        let block = &p.blocks[1];
        let mut progress = 0.0;
        for point in block.path.iter().step_by(3) {
            let current = block.fraction_after(*point, progress);
            assert!(current + 1e-6 >= progress);
            progress = current;
        }
        assert!(
            progress > 0.98,
            "Repeated arc positions must not return progress to the first turn"
        );
    }
    #[test]
    fn freecad_xy_arcs_accept_unused_k() {
        let p = program(include_str!("../examples/freecad-lead-in.nc"));
        assert!(p.uploaded.contains("I-2.121J-2.121K0F1500.000"));
        assert!((p.blocks.last().unwrap().path.last().unwrap().x - 19.9).abs() < 1e-9);
        assert!(parse("G1X1K0F60", ParseOptions::default()).is_err());
        assert!(parse("G81X1Z-1R2F60", ParseOptions::default()).is_err());
    }
    #[test]
    fn compact_positions_and_g_codes_keep_other_precision() {
        let p = program(
            "G021G090\nG00X100.00001Y-0.00001Z5.123456\nG01X101.23456F60.123456\nG04P0.123456",
        );
        assert_eq!(p.blocks[0].code, "G21G90");
        assert_eq!(p.blocks[1].code, "G0X100Y0Z5.123");
        assert_eq!(p.blocks[2].code, "G1X101.235F60.123456");
        assert_eq!(p.blocks[3].code, "G4P0.123456");
        assert_eq!(p.blocks[2].path.last().unwrap().x, 101.235);
    }
    #[test]
    fn convex_hull_removes_interior_points_and_includes_arc_extrema() {
        let p = program("G0X10Y0\nG1X10Y10F600\nX5Y5\nX0Y10\nX0Y0");
        let hull = p.hull();
        assert_eq!(hull.len(), 4);
        assert!(!hull.iter().any(|p| p.x == 5.0 && p.y == 5.0));
        let p = program("G0X7Y1\nG3X7Y1I-2J-1F600");
        let hull = p.hull();
        let radius = 5.0f64.sqrt();
        assert!(
            (hull.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max) - (5.0 + radius)).abs()
                < 1e-9
        );
        assert!((hull.iter().map(|p| p.y).fold(f64::INFINITY, f64::min) + radius).abs() < 1e-9);
    }
}
