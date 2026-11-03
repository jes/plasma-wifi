use anyhow::{Context, Result, ensure};
use std::{collections::BTreeMap, fs, io::Write, path::PathBuf, time::Instant};

pub const PROBE: &str = "$J=G21G91X0F1P1";

pub struct JogOverride {
    pub saved: u8,
    pub target: u8,
    pub sent: Option<Instant>,
    pub restoring: bool,
    pub warned: bool,
    pub confirmed: bool,
}
impl JogOverride {
    pub fn interrupted(saved: u8) -> Self {
        Self {
            saved,
            target: saved,
            sent: None,
            restoring: true,
            warned: false,
            confirmed: false,
        }
    }
    pub fn interrupt(&mut self) {
        self.restoring = true;
        self.target = self.saved;
        self.sent = None;
        self.warned = false;
        self.confirmed = false;
    }
}

/// The shared feed register must survive an interrupted jog or closed sender.
/// Store the original value before changing it, separately for each controller.
#[derive(Default)]
pub struct JogRecovery {
    path: Option<PathBuf>,
    saved: BTreeMap<String, u8>,
}
impl JogRecovery {
    pub fn load(path: PathBuf) -> Result<Self> {
        let saved: BTreeMap<String, u8> = match fs::read(&path) {
            Ok(bytes) => {
                serde_json::from_slice(&bytes).context("Invalid jog override recovery state")?
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => BTreeMap::new(),
            Err(e) => return Err(e).context("Read jog override recovery state"),
        };
        ensure!(
            saved.values().all(|feed| (10..=200).contains(feed)),
            "Invalid saved jog feed override"
        );
        Ok(Self {
            path: Some(path),
            saved,
        })
    }
    pub fn get(&self, address: &str) -> Option<u8> {
        self.saved.get(address).copied()
    }
    pub fn begin(&mut self, address: &str, feed: u8) -> Result<()> {
        self.saved.insert(address.into(), feed);
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
        let parent = path.parent().context("Jog recovery directory missing")?;
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
