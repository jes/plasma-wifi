use crate::gcode::Point;
use anyhow::{Context, Result, ensure};
use std::{collections::BTreeMap, fs, io::Write, path::PathBuf};

/// Host-side checkpoints, separate from FluidNC's persistent offsets: a reboot
/// resets its machine position, so saving only G54's offset is insufficient.
#[derive(Default)]
pub struct Coordinates {
    path: Option<PathBuf>,
    positions: BTreeMap<String, Point>,
}
impl Coordinates {
    pub fn path() -> PathBuf {
        std::env::var_os("XDG_STATE_HOME")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".local/state")
            })
            .join("plasma-wifi/coordinates.json")
    }
    pub fn load(path: PathBuf) -> Result<Self> {
        let positions: BTreeMap<String, Point> = match fs::read(&path) {
            Ok(bytes) => serde_json::from_slice(&bytes).context("Invalid coordinate checkpoint")?,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => BTreeMap::new(),
            Err(e) => return Err(e).context("Read coordinate checkpoint"),
        };
        ensure!(
            positions.values().all(|p| finite(*p)),
            "Non-finite coordinate checkpoint"
        );
        Ok(Self {
            path: Some(path),
            positions,
        })
    }
    pub fn get(&self, address: &str) -> Option<Point> {
        self.positions.get(address).copied()
    }
    pub fn record(&mut self, address: &str, position: Point) -> Result<()> {
        ensure!(finite(position), "Cannot save non-finite coordinates");
        if self.get(address) == Some(position) {
            return Ok(());
        }
        self.positions.insert(address.to_owned(), position);
        self.flush()
    }
    pub fn flush(&self) -> Result<()> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        let parent = path
            .parent()
            .context("Coordinate checkpoint directory missing")?;
        fs::create_dir_all(parent)?;
        let tmp = path.with_extension("tmp");
        let mut file = fs::File::create(&tmp)?;
        file.write_all(&serde_json::to_vec_pretty(&self.positions)?)?;
        file.sync_all()?;
        fs::rename(tmp, path)?;
        fs::File::open(parent)?.sync_all()?;
        Ok(())
    }
}
fn finite(p: Point) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}
