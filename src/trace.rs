use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, fs, io::Write, path::PathBuf};

#[derive(Clone, Serialize, Deserialize)]
pub struct SavedTrace {
    pub feed: u8,
    pub modal: String,
}

/// Persist the borrowed controller state before changing it, so a closed app
/// or lost connection cannot silently leave the next cutting job at trace feed.
#[derive(Default)]
pub struct TraceRecovery {
    path: Option<PathBuf>,
    saved: BTreeMap<String, SavedTrace>,
}
impl TraceRecovery {
    pub fn load(path: PathBuf) -> Result<Self> {
        let saved = match fs::read(&path) {
            Ok(bytes) => serde_json::from_slice(&bytes).context("Invalid trace recovery state")?,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => BTreeMap::new(),
            Err(e) => return Err(e).context("Read trace recovery state"),
        };
        ensure!(saved.values().all(|s: &SavedTrace| (10..=200).contains(&s.feed) && valid_modal_restore(&s.modal)),
            "Invalid saved trace feed or modal state");
        Ok(Self {
            path: Some(path),
            saved,
        })
    }
    pub fn get(&self, address: &str) -> Option<SavedTrace> {
        self.saved.get(address).cloned()
    }
    pub fn begin(&mut self, address: &str, saved: SavedTrace) -> Result<()> {
        self.saved.insert(address.into(), saved);
        self.flush()
    }
    pub fn finish(&mut self, address: &str) -> Result<()> {
        self.saved.remove(address);
        self.flush()
    }
    fn flush(&self) -> Result<()> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        let parent = path.parent().context("Trace recovery directory missing")?;
        fs::create_dir_all(parent)?;
        let tmp = path.with_extension("tmp");
        let mut file = fs::File::create(&tmp)?;
        file.write_all(&serde_json::to_vec_pretty(&self.saved)?)?;
        file.sync_all()?;
        fs::rename(tmp, path)?;
        fs::File::open(parent)?.sync_all()?;
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub enum RestorePhase {
    Tracing,
    WaitingReset,
    WaitingIdle,
    ModalQueued,
    Override { sent: std::time::Instant },
}

pub struct TraceRestore {
    pub saved: SavedTrace,
    pub phase: RestorePhase,
    pub outcome: String,
}
impl TraceRestore {
    pub fn interrupted(saved: SavedTrace) -> Self {
        Self {
            saved,
            phase: RestorePhase::WaitingReset,
            outcome: "Hull trace cancelled".into(),
        }
    }
}

pub struct Trace {
    pub points: Vec<crate::gcode::Point>,
    pub next: usize,
    pub sent: Option<std::time::Instant>,
    pub nominal_feed: f64,
    pub override_percent: u8,
    pub override_sent: Option<std::time::Instant>,
    pub prepared: bool,
}

// A recovery file may only restore the modes we borrowed. In particular it
// must not accept motion such as G28 or any accessory command.
fn valid_modal_restore(source: &str) -> bool {
    let Some(source) = source
        .strip_prefix("G21")
        .or_else(|| source.strip_prefix("G20"))
    else {
        return false;
    };
    let Some(source) = source
        .strip_prefix("G90")
        .or_else(|| source.strip_prefix("G91"))
    else {
        return false;
    };
    let Some(source) = source
        .strip_prefix("G94")
        .or_else(|| source.strip_prefix("G93"))
    else {
        return false;
    };
    ["G0F", "G1F", "G2F", "G3F"].iter().any(|prefix| {
        source
            .strip_prefix(prefix)
            .and_then(|feed| feed.parse::<f64>().ok())
            .is_some_and(|feed| feed.is_finite() && feed >= 0.0)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn recovery_cannot_introduce_axes_accessories_or_nonmodal_motion() {
        assert!(valid_modal_restore("G20G91G94G1F12.00000000"));
        for source in [
            "G21G90G94G28F600",
            "G21G90G94G1F600X100",
            "G21G90G94G0F0M3",
            "G21G90G94G0FNaN",
        ] {
            assert!(!valid_modal_restore(source), "{source}");
        }
    }
}
