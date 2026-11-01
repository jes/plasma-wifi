# Plasma WiFi

A Linux desktop controller for an XY plasma cutter running FluidNC 4.1.1. Jobs are uploaded to LocalFS as `plasma.nc`, verified by size and a complete download comparison, then executed on the controller. Motion does not depend on streaming from the laptop.

## Run

Copy `dist/plasma-wifi` to an x86-64 Linux laptop and run it:

```sh
chmod +x plasma-wifi
./plasma-wifi
```

The executable is statically linked with musl. It needs an X11 display, or XWayland on a Wayland desktop. It does not load host glibc, libX11, GTK, Qt, or OpenGL. Rendering, the window icon, and bundled fonts are built into the binary. File selection uses the desktop portal, with zenity/kdialog as fallbacks on older desktops; these are separate desktop services, with no GTK/Qt libraries linked or loaded by the app. A desktop needs one of those existing file-picker services for the Open button; passing a filename on the command line also works.

The default controller is `192.168.0.1:23`, with HTTP on port 80. Change these in Settings or use:

```sh
./plasma-wifi --controller 192.168.0.1:23 --http http://192.168.0.1 job.nc
```

Install the supplied `config.yaml` using **Files → Upload file…** (or the FluidNC WebUI) before using active-line tracking: it adds `use_line_numbers: true` and retains the existing motor configuration, including the unused Z pins. Restart the controller to load the uploaded configuration. The app does not install controller configuration automatically.

## Use

- Open a G-code file with **Open G-code**, **O**, or **Ctrl+O**. Processing, uploading, and verification happen automatically. The header shows progress and whether the file is verified. **Run** or **R** starts it explicitly.
- **Files** lists files in the controller's LocalFS root, including `config.yaml`. **Upload file…** selects a local file and uploads its exact bytes under the same filename, replacing that file if it exists; the size and downloaded contents are verified. Select a listed file (or type its name), then **Download…** to save a local copy through the system save dialog. A failed download leaves an existing local backup intact. Transfers require an idle controller or Alarm with the torch off, and block motion controls until finished. Configuration uploads take effect after a controller restart; they do not restart it automatically. Uploading `plasma.nc` through Files invalidates the selected job; reopen it with **Open G-code** to process and verify it before Run.
- Jog using the X/Y buttons or arrow keys, with pugsender-style per-axis targets: taps accumulate the selected increment, while holding a direction for 150 ms extends temporary travel. Releasing a held direction immediately cancels that axis’s remaining step and queued travel. Genuine taps on other axes and directions still held continue from the confirmed stopped position. A simple tap finishes its step. Holding **Shift** temporarily multiplies the speed by four, capped at 3000 mm/min, for both mouse and keyboard jogging. Speed changes cancel and reissue unfinished motion at the new feed. **F** focuses jog feed and **I** focuses jog increment; type a number and press **Enter** to apply. **Ctrl-click** the toolpath to jog to that X/Y work position. Keyboard jogging is suppressed in text fields and dialogs. Escape, focus loss, Stop, or an expired UI heartbeat cancels all manual jogging. Commands explicitly select millimetres and absolute positioning; their targets do not depend on OS key repeat or the controller's current units and distance mode.
- **Trace hull at jog speed** follows the job’s convex hull with torch-off, normal XY `G1` motion, closes the outline, then returns to the starting work position. Its programmed feed is four times the selected jog feed: normal tracing uses **25%** realtime feed override, and holding **Shift** selects **100%**, so queued motion changes speed without cancellation. Machine rate and acceleration limits still apply. The app saves the user's feed override and parser modes before tracing, restores them on completion or cancellation, and keeps motion controls blocked until override restoration is confirmed. Feed override buttons are disabled during tracing. Cancellation holds motion, waits for it to stop, then resets the controller to discard the remaining planner blocks. The preview’s **Hull** checkbox shows the outline. Use **Cancel hull trace** or **Stop** to cancel. Tracing requires an idle machine with the torch off and cannot overlap a job, upload, or manual jog. It includes rapid travel as well as feed moves, including the approach from the current position; arc samples include exact X/Y extrema. Connection loss cancels the trace without replaying it, and cleanup resumes after reconnection. Saved trace state survives app restarts so an interrupted trace cannot leave the next cutting job at the trace override. This is a physical travel check: the app does not know the machine’s mechanical limits.
- **X0**, **Y0**, and **/2** change work coordinates without moving. Click a DRO number or press **X**/**Y**, edit it, and press **Enter** to apply. Typing **/2** halves that coordinate; F/I accept numeric values only. These shortcuts leave already focused inputs alone. The app uses `G54` and `G10 L20 P1` for work offsets; displayed coordinates are always millimetres.
- Below the DRO, **Velocity** shows FluidNC's realtime tool speed in mm/min, including feed overrides; unavailable or stale reports show a dash. **Reported modes** shows the latest controller parser modes, including G/M codes and tool selection, from `$G` and automatic modal reports. Parser lookahead can be ahead of physical motion, and FluidNC can suppress motion-only modal changes while moving. These readouts use [FluidNC's report formats](https://github.com/bdring/FluidNC/blob/v4.1.1/FluidNC/src/Report.cpp) and [automatic modal reporting](https://github.com/bdring/FluidNC/blob/v4.1.1/FluidNC/src/Channel.cpp).
- Feed controls adjust the controller's realtime feed override. CAM-authored dwells are unaffected.
- Set material-dependent pierce delays in CAM using `G4 P<seconds>`. The sender preserves those dwells and includes them in timing.
- **Pause** holds motion, then extinguishes a lit torch. **Resume** restores it only if it was lit, then resumes once restoration is confirmed, without adding a delay. **Stop** holds motion, extinguishes the torch, then soft-resets the controller after the hold completes. This avoids the `ALARM:3` that FluidNC raises when reset interrupts motion. Pressing Stop again forces an immediate reset; a hold timeout also falls back to reset. Genuine alarms remain visible until the controller leaves Alarm, and are retained in the console history. A later Run starts the file again from the beginning.
- **P** pauses a running job; either **R** or **P** resumes once the paused torch-off state is confirmed. **S** stops. Shortcut letters appear on the buttons. Plain-letter shortcuts leave focused inputs and dialogs alone.
- **Console** shows FluidNC commands and responses separately from the internal **Log**. Press **G** or **M** outside an input to open and focus it with that letter ready to type. Type a command and press **Enter** or **Send**; **Up/Down** recalls command history. **Escape** closes the console without submitting the draft. Status reports can be shown or hidden; hidden reports do not displace commands and replies, and long console lines scroll horizontally. Ordinary commands require Idle/Alarm, because the LocalFS Run acknowledgement is deferred until the file finishes; `?`, `!`, and `~` remain available during jobs and use the app’s pause/resume behavior.
- DRO and override updates use 100 ms controller auto reports, plus a refresh after acknowledged commands and realtime overrides. A slower heartbeat checks that the connection is still alive.
- While disconnected, a separate background helper requests `nmcli device wifi rescan` every 100 ms, with at most one command running. NetworkManager schedules the actual radio scans and the OS's saved Wi-Fi settings control reconnection. A command still running after five seconds is killed; missing `nmcli` disables the helper and adds a Log message. Requests stop when the controller connects, and demo mode never runs the helper. This reuses the laptop's existing NetworkManager tools without adding a linked library dependency.
- The G-code panel opens on **Original**, and can switch between original and uploaded lines. Both views color each letter and its value together, with gray comments and a separate number margin; Original line numbers are subdued. Hold **Ctrl** and scroll anywhere in the application to resize the entire UI, including all text, buttons, coordinate displays, and dialogs; the size is remembered between sessions. Auto-scroll follows the active motion line; FluidNC may omit line numbers during a dwell or between planner blocks, so the most recent known line remains highlighted.
- The preview uses cyan rapid moves, yellow feed moves, and distinct active/completed segments. Drag to pan, wheel to zoom, and **Fit** to reset the view. On shorter windows, the lower controls scroll while the DRO and Run/Pause/Stop stay visible.

Manual jogging reads the controller’s X/Y acceleration and planner capacity on connection and reset. Its target lookahead is the directional braking distance plus a margin for status age and measured command latency. Packet lengths grow when needed to fit that distance into the planner or sustain motion over a slower connection. Per-axis incremental destinations are independent of held travel; released held axes stop while other axes can continue. Stationary, uncommanded axes reset their local destinations to the reported position after a short grace period. The worker owns all jog state, checks command acknowledgements, and reserves at least two planner slots when buffer reports are available. Slower acceleration or communication increases the time needed to stop and restart a manual jog. Hull tracing sends successive full edges through the normal motion planner and uses realtime feed override for speed changes.

The latest reported X/Y/Z work position is saved per controller TCP address and restored automatically after reconnection, app restarts, FluidNC reboots, or both. This assumes the toolhead has not moved since its last saved report. Restoration changes offsets without moving or homing, and is verified before jogging or Run is enabled. Existing matching coordinates need no write; running, jogging, or held controllers keep their live coordinates. If FluidNC is in Alarm, use **Unlock alarm** before restoration can proceed. A rejected or unverified restoration blocks motion controls and preserves the checkpoint for another reconnect. FluidNC's machine coordinates (`G53`) still reset on a firmware reboot; restoration preserves the work coordinates used by the DRO and `G54` jobs.

Files cannot be replaced during running or paused jobs. Run is gated on verified contents and fresh idle controller status. Reconnection queries state without replaying Run, manual coordinate edits, or jogs. Elapsed time is marked uncertain after a connection gap because an unobserved pause or override change cannot be reconstructed exactly.

## G-code and timing

The processor handles linear moves, XY `G2/G3` arcs with I/J or signed R, full circles with explicit X/Y endpoints, helical Z coordinates, arc P rotations, G20/G21 units, G90/G91 positioning, G91.1 incremental arc centers, G94 feed, G4 dwell, G53 machine moves, G54, common FreeCAD Grbl preamble words, M3/M5, and M2/M30. Positional words (`X/Y/Z/I/J/K/R`) are rounded to at most three decimal places in the file’s units, trailing zeroes are removed, and G codes use canonical numbers (`G00` → `G0`, `G01` → `G1`). Geometry and timing use those same rounded values. Feed, spindle, and dwell precision is retained. FreeCAD’s unused `K0` on XY arcs is accepted, as it is by FluidNC. CAM-authored dwells and combined ignition/motion blocks are preserved without inserting delays. Original line numbers are replaced and mapped to their source lines.

The UI has no Z controls. Existing Z commands are retained and explicitly flagged, so a milling post's retract moves are never silently deleted. Use an XY/plasma CAM setup for this machine. Unsupported coordinate systems, planes, expressions, probing, canned cycles, and other unmodelled commands produce a source-line error and are not uploaded. FreeCAD's Grbl post normally disables tool-length compensation; leave it disabled. A representative Grbl preamble is covered by the parser tests.

Timing separates feed moves, axis-limited rapid moves, and fixed dwells. Cumulative times and projection of the reported position onto the active toolpath locate progress. Historical feed overrides normalize completed work before calibrating the motion-time coefficient; the current override scales future feed work. Dwells are accounted for separately, and paused time is excluded. Estimates remain approximate because of acceleration, planner blending, polling latency, and unsupported hardware feedback such as arc-established sensing.

Pause/resume uses FluidNC's realtime feed hold (`!`), spindle-stop override (`0x9e`) while held, and cycle start (`~`). Torch state is confirmed from accessory reports before the app proceeds. Motion resumes after the restored torch is reported on; no pierce delay is added. These semantics were checked against the [4.1.1 suspend and spindle-management implementation](https://github.com/bdring/FluidNC/blob/v4.1.1/FluidNC/src/Protocol.cpp) and [parking spindle restoration](https://github.com/bdring/FluidNC/blob/v4.1.1/FluidNC/src/Parking.cpp).

## Build and check

```sh
sudo apt install musl-tools
rustup target add x86_64-unknown-linux-musl
./scripts/build.sh
cargo test --locked --release --target x86_64-unknown-linux-musl
```

Cargo fetches the Rust dependencies. `Cargo.lock` pins them. Settings are saved under `$XDG_CONFIG_HOME/plasma-wifi/settings.json`, or `~/.config/plasma-wifi/settings.json`. Pending hull-trace override and modal restoration is saved per controller in `trace-restore.json` alongside coordinate checkpoints. Coordinate checkpoints are written atomically under `$XDG_STATE_HOME/plasma-wifi/coordinates.json`, or `~/.local/state/plasma-wifi/coordinates.json`, as reports arrive. Demo mode keeps coordinates in memory and does not read or write these checkpoints.

To explore without hardware:

```sh
./dist/plasma-wifi --demo
```

Demo starts a loopback-only TCP/HTTP simulator, loads the rounded-plate example, and does not save settings. The simulator supports upload, verification, jog acceleration and braking, queued-jog cancellation, motion, hold/restore, sparse status fields, auto reports, overrides, absolute/relative jogs, work offsets, reset-during-motion alarms, and connection drops. Automated tests cover controller interactions against it; this is separate from testing a physical cutter.

The GUI smoke check additionally needs Xvfb, xdotool, xset, xclip, zenity, Python Pillow and python-xlib on the development machine:

```sh
xvfb-run -a -s '-screen 0 1280x1100x24' python3 scripts/ui_smoke.py
```
