use std::{
    ffi::OsString,
    process::{Child, Command, Stdio},
    sync::mpsc::{self, Receiver, RecvTimeoutError, Sender},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

const REQUEST_INTERVAL: Duration = Duration::from_millis(100);
const COMMAND_TIMEOUT: Duration = Duration::from_secs(5);

pub struct Rescan {
    stop: Sender<()>,
    handle: Option<JoinHandle<()>>,
}

impl Rescan {
    pub fn start(
        disconnected: impl Fn() -> bool + Send + 'static,
        report: impl Fn(String) + Send + 'static,
    ) -> Self {
        Self::spawn("nmcli".into(), COMMAND_TIMEOUT, disconnected, report)
    }

    fn spawn(
        command: OsString,
        timeout: Duration,
        disconnected: impl Fn() -> bool + Send + 'static,
        report: impl Fn(String) + Send + 'static,
    ) -> Self {
        let (stop, rx) = mpsc::channel();
        let handle = thread::spawn(move || {
            run(rx, command, timeout, disconnected, report);
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Rescan {
    fn drop(&mut self) {
        let _ = self.stop.send(());
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn run(
    stop: Receiver<()>,
    command: OsString,
    timeout: Duration,
    disconnected: impl Fn() -> bool,
    report: impl Fn(String),
) {
    let mut running: Option<(Child, Instant, bool)> = None;
    loop {
        let scan = disconnected();
        if let Some((child, started, killed)) = &mut running {
            match child.try_wait() {
                Ok(Some(_)) => running = None,
                Ok(None) => {
                    if !*killed && (!scan || started.elapsed() >= timeout) {
                        if let Err(error) = child.kill()
                            && !matches!(child.try_wait(), Ok(Some(_)))
                        {
                            report(format!("Could not stop WiFi rescan: {error}"));
                            break;
                        }
                        *killed = true;
                        if scan {
                            report("WiFi rescan command timed out; killed nmcli".into());
                        }
                    }
                }
                Err(error) => {
                    report(format!("Could not check WiFi rescan process: {error}"));
                    break;
                }
            }
        }
        if scan && running.is_none() {
            match Command::new(&command)
                .args(["device", "wifi", "rescan"])
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
            {
                Ok(child) => running = Some((child, Instant::now(), false)),
                Err(error) => {
                    report(format!(
                        "Automatic WiFi rescans unavailable: could not start nmcli: {error}"
                    ));
                    break;
                }
            }
        }
        // nmcli acknowledges the request before the radio scan completes.
        // NetworkManager schedules/coalesces scans; we keep requests prompt.
        // Busy/rate-limit exit statuses are expected and do not become UI errors.
        match stop.recv_timeout(REQUEST_INTERVAL) {
            Err(RecvTimeoutError::Timeout) => {}
            Ok(()) | Err(RecvTimeoutError::Disconnected) => break,
        }
    }
    if let Some((mut child, _, _)) = running {
        let _ = child.kill();
        // Reap outside this supervisor so even an uninterruptible child cannot
        // hold up controller shutdown or prevent the application from closing.
        thread::spawn(move || {
            let _ = child.wait();
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        fs,
        os::unix::fs::PermissionsExt,
        path::PathBuf,
        sync::{
            Arc, Mutex,
            atomic::{AtomicBool, AtomicU64, Ordering},
        },
    };

    struct FakeNmcli {
        directory: PathBuf,
        command: PathBuf,
    }
    impl FakeNmcli {
        fn new(body: &str) -> Self {
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let directory = std::env::temp_dir().join(format!(
                "plasma-wifi-rescan-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&directory).unwrap();
            let command = directory.join("nmcli");
            fs::write(&command, format!("#!/bin/sh\nprintf '%s\\n' \"$$\" > \"$0.pid\"\nprintf '%s\\n' \"$*\" >> \"$0.calls\"\n{body}\n")).unwrap();
            fs::set_permissions(&command, fs::Permissions::from_mode(0o755)).unwrap();
            Self { directory, command }
        }
        fn calls(&self) -> Vec<String> {
            fs::read_to_string(self.command.with_extension("calls"))
                .unwrap_or_default()
                .lines()
                .map(str::to_owned)
                .collect()
        }
        fn pid(&self) -> Option<u32> {
            fs::read_to_string(self.command.with_extension("pid"))
                .ok()?
                .trim()
                .parse()
                .ok()
        }
    }
    impl Drop for FakeNmcli {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.directory);
        }
    }
    fn wait(mut condition: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(3);
        while !condition() {
            assert!(Instant::now() < deadline, "WiFi helper condition timed out");
            thread::sleep(Duration::from_millis(5));
        }
    }
    fn alive(pid: u32) -> bool {
        PathBuf::from(format!("/proc/{pid}")).exists()
    }

    #[test]
    fn requests_promptly_only_while_disconnected_and_restarts_after_loss() {
        let fake = FakeNmcli::new("exit 0");
        let disconnected = Arc::new(AtomicBool::new(false));
        let state = disconnected.clone();
        let rescan = Rescan::spawn(
            fake.command.clone().into(),
            COMMAND_TIMEOUT,
            move || state.load(Ordering::Relaxed),
            |_| {},
        );
        thread::sleep(Duration::from_millis(150));
        assert!(fake.calls().is_empty());
        disconnected.store(true, Ordering::Relaxed);
        wait(|| fake.calls().len() >= 3);
        assert!(fake.calls().iter().all(|args| args == "device wifi rescan"));
        disconnected.store(false, Ordering::Relaxed);
        thread::sleep(Duration::from_millis(200));
        let count = fake.calls().len();
        thread::sleep(Duration::from_millis(200));
        assert_eq!(fake.calls().len(), count);
        disconnected.store(true, Ordering::Relaxed);
        wait(|| fake.calls().len() > count);
        drop(rescan);
    }

    #[test]
    fn hung_command_never_overlaps_and_is_killed_on_connection_and_shutdown() {
        let fake = FakeNmcli::new("exec sleep 30");
        let disconnected = Arc::new(AtomicBool::new(true));
        let state = disconnected.clone();
        let rescan = Rescan::spawn(
            fake.command.clone().into(),
            COMMAND_TIMEOUT,
            move || state.load(Ordering::Relaxed),
            |_| {},
        );
        wait(|| !fake.calls().is_empty() && fake.pid().is_some());
        let first = fake.pid().unwrap();
        thread::sleep(Duration::from_millis(250));
        assert_eq!(fake.calls().len(), 1);
        assert!(alive(first));
        disconnected.store(false, Ordering::Relaxed);
        wait(|| !alive(first));
        disconnected.store(true, Ordering::Relaxed);
        wait(|| fake.calls().len() == 2 && fake.pid() != Some(first));
        let second = fake.pid().unwrap();
        let started = Instant::now();
        drop(rescan);
        assert!(started.elapsed() < Duration::from_secs(1));
        wait(|| !alive(second));
    }

    #[test]
    fn command_deadline_kills_a_stuck_process() {
        let fake = FakeNmcli::new("exec sleep 30");
        let disconnected = Arc::new(AtomicBool::new(true));
        let state = disconnected.clone();
        let messages = Arc::new(Mutex::new(Vec::new()));
        let notices = messages.clone();
        let rescan = Rescan::spawn(
            fake.command.clone().into(),
            Duration::from_millis(150),
            move || state.load(Ordering::Relaxed),
            move |message| notices.lock().unwrap().push(message),
        );
        wait(|| !fake.calls().is_empty() && fake.pid().is_some());
        let pid = fake.pid().unwrap();
        wait(|| {
            messages
                .lock()
                .unwrap()
                .iter()
                .any(|message| message.contains("timed out"))
        });
        disconnected.store(false, Ordering::Relaxed);
        wait(|| !alive(pid));
        drop(rescan);
    }

    #[test]
    fn missing_nmcli_reports_once_and_finishes() {
        let fake = FakeNmcli::new("exit 0");
        let messages = Arc::new(Mutex::new(Vec::new()));
        let notices = messages.clone();
        let rescan = Rescan::spawn(
            fake.directory.join("missing-nmcli").into(),
            COMMAND_TIMEOUT,
            || true,
            move |message| notices.lock().unwrap().push(message),
        );
        wait(|| rescan.handle.as_ref().unwrap().is_finished());
        assert_eq!(messages.lock().unwrap().len(), 1);
        assert!(messages.lock().unwrap()[0].contains("unavailable"));
    }
}
