//! Use the desktop's file chooser without linking a GUI toolkit into the binary.
use anyhow::{Result, bail};
use ashpd::desktop::{
    ResponseError,
    file_chooser::{FileFilter, SelectedFiles},
};
use std::{
    path::{Path, PathBuf},
    process::Command,
    sync::mpsc::{self, Receiver},
    thread,
};

#[derive(Clone)]
pub enum Purpose {
    Gcode,
    Upload,
    Download(String),
}
impl Purpose {
    fn title(&self) -> &'static str {
        match self {
            Self::Gcode => "Open G-code",
            Self::Upload => "Upload controller file",
            Self::Download(_) => "Download controller file",
        }
    }
}

pub fn open(folder: PathBuf, window: u32) -> Receiver<Result<Option<PathBuf>>> {
    choose(folder, window, Purpose::Gcode)
}

pub fn choose(folder: PathBuf, window: u32, purpose: Purpose) -> Receiver<Result<Option<PathBuf>>> {
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let result = futures_lite::future::block_on(portal(&folder, window, &purpose));
        let result = match result {
            Ok(path) => Ok(path),
            Err(portal_error) => helper(&folder, window, &purpose).map_err(|e| {
                anyhow::anyhow!(
                    "Could not open the system file chooser: {e:#}. Desktop portal: {portal_error}"
                )
            }),
        };
        let _ = tx.send(result);
    });
    rx
}

async fn portal(folder: &Path, window: u32, purpose: &Purpose) -> ashpd::Result<Option<PathBuf>> {
    let identifier = ashpd::WindowIdentifier::from(ashpd::WindowIdentifierType::X11(window.into()));
    let result = if let Purpose::Download(name) = purpose {
        SelectedFiles::save_file()
            .identifier(identifier)
            .title(purpose.title())
            .accept_label("Save")
            .modal(true)
            .current_folder(folder)?
            .current_name(name.as_str())
            .send()
            .await
            .and_then(|request| request.response())
    } else {
        let mut request = SelectedFiles::open_file()
            .identifier(identifier)
            .title(purpose.title())
            .accept_label("Open")
            .modal(true)
            .multiple(false);
        if matches!(purpose, Purpose::Gcode) {
            request = request.filter(
                FileFilter::new("G-code")
                    .glob("*.nc")
                    .glob("*.gcode")
                    .glob("*.ngc")
                    .glob("*.tap"),
            );
        }
        request
            .filter(FileFilter::new("All files").glob("*"))
            .current_folder(folder)?
            .send()
            .await
            .and_then(|request| request.response())
    };
    match result {
        Ok(files) => Ok(files
            .uris()
            .first()
            .and_then(|uri| url::Url::parse(uri.as_str()).ok()?.to_file_path().ok())),
        Err(ashpd::Error::Response(ResponseError::Cancelled)) => Ok(None),
        Err(error) => Err(error),
    }
}

fn helper(folder: &Path, window: u32, purpose: &Purpose) -> Result<Option<PathBuf>> {
    let kde = std::env::var("XDG_CURRENT_DESKTOP")
        .unwrap_or_default()
        .to_ascii_lowercase()
        .contains("kde");
    let tools = if kde {
        ["kdialog", "zenity"]
    } else {
        ["zenity", "kdialog"]
    };
    for tool in tools {
        let mut command = Command::new(tool);
        let path = if let Purpose::Download(name) = purpose {
            folder.join(name)
        } else {
            folder.join("")
        };
        if tool == "zenity" {
            command
                .args(["--file-selection", "--modal"])
                .arg(format!("--title={}", purpose.title()))
                .arg(format!("--attach={window}"))
                .arg(format!("--filename={}", path.display()));
            if matches!(purpose, Purpose::Download(_)) {
                command.args(["--save", "--confirm-overwrite"]);
            }
            if matches!(purpose, Purpose::Gcode) {
                command.arg("--file-filter=G-code | *.nc *.gcode *.ngc *.tap");
            }
            command.arg("--file-filter=All files | *");
        } else {
            command
                .args(["--attach", &window.to_string(), "--title", purpose.title()])
                .arg(if matches!(purpose, Purpose::Download(_)) {
                    "--getsavefilename"
                } else {
                    "--getopenfilename"
                })
                .arg(path)
                .arg(if matches!(purpose, Purpose::Gcode) {
                    "G-code (*.nc *.gcode *.ngc *.tap);;All files (*)"
                } else {
                    "All files (*)"
                });
        }
        match command.output() {
            Ok(output) if output.status.success() => {
                let path = String::from_utf8(output.stdout)?;
                return Ok(if path.trim_end_matches(['\n', '\r']).is_empty() {
                    None
                } else {
                    Some(PathBuf::from(path.trim_end_matches(['\n', '\r'])))
                });
            }
            Ok(output) if output.status.code() == Some(1) => return Ok(None),
            Ok(_) | Err(_) => continue,
        }
    }
    bail!("no working desktop portal, zenity or kdialog is available")
}
