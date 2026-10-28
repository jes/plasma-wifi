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

Install the supplied `config.yaml` through the FluidNC WebUI before using active-line tracking: it adds `use_line_numbers: true` and retains the existing motor configuration, including the unused Z pins. The app does not install controller configuration automatically.

## Use

- Open a G-code file with **Open G-code** or **Ctrl+O**. Processing, uploading, and verification happen automatically. The header shows progress and whether the file is verified. **Run** starts it explicitly.
- Jog using held on-screen buttons or arrow keys. The speed is in mm/min. Keyboard jogging is suppressed in text fields and dialogs, and ends on key release or focus loss. Short jog segments require a continuing UI heartbeat.
- **Trace hull at jog speed** follows the job’s convex hull using torch-off XY jog moves at the selected speed, closes the outline, then returns to the starting work position. The preview’s **Hull** checkbox shows that outline. Use **Cancel hull trace** or **Stop** to cancel. Tracing requires an idle machine with the torch off and cannot overlap a job, upload, or manual jog. It includes rapid travel as well as feed moves, including the approach from the current position; arc samples include exact X/Y extrema. Connection loss cancels the trace without replaying it; an in-flight jog may continue until cancellation reaches the controller. This is a physical travel check: the app does not know the machine’s mechanical limits.
- **X0**, **Y0**, and **/2** change work coordinates without moving. Click a DRO number, edit it, and press **Enter** to apply. The app uses `G54` and `G10 L20 P1` for work offsets; displayed coordinates are always millimetres.
- Feed controls adjust the controller's realtime feed override. CAM-authored dwells are unaffected.
- Set material-dependent pierce delays in CAM using `G4 P<seconds>`. The sender preserves those dwells and includes them in timing.
- **Pause** holds motion, then extinguishes a lit torch. **Resume** restores it only if it was lit, then resumes once restoration is confirmed, without adding a delay. **Stop** holds motion, extinguishes the torch, then soft-resets the controller after the hold completes. This avoids the `ALARM:3` that FluidNC raises when reset interrupts motion. Pressing Stop again forces an immediate reset; a hold timeout also falls back to reset. Genuine alarms remain visible until the controller leaves Alarm, and are retained in the console history. A later Run starts the file again from the beginning.
- **Console** shows FluidNC commands and responses separately from the internal **Log**. Type a command and press **Enter** or **Send**; **Up/Down** recalls command history. Status reports can be shown or hidden. Ordinary commands require Idle/Alarm, because the LocalFS Run acknowledgement is deferred until the file finishes; `?`, `!`, and `~` remain available during jobs and use the app’s pause/resume behavior.
- DRO and override updates use 100 ms controller auto reports, plus a refresh after acknowledged commands and realtime overrides. A slower heartbeat checks that the connection is still alive.
- The G-code panel switches between original and uploaded lines. Auto-scroll follows the active motion line; FluidNC may omit line numbers during a dwell or between planner blocks, so the most recent known line remains highlighted.
- The preview uses subdued cyan rapid moves, prominent orange feed moves, and distinct active/completed segments. Drag to pan, wheel to zoom, and **Fit** to reset the view.

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

Cargo fetches the Rust dependencies. `Cargo.lock` pins them. Settings are saved under `$XDG_CONFIG_HOME/plasma-wifi/settings.json`, or `~/.config/plasma-wifi/settings.json`. Coordinate checkpoints are written atomically under `$XDG_STATE_HOME/plasma-wifi/coordinates.json`, or `~/.local/state/plasma-wifi/coordinates.json`, as reports arrive. Demo mode keeps coordinates in memory and does not read or write these checkpoints.

To explore without hardware:

```sh
./dist/plasma-wifi --demo
```

Demo starts a loopback-only TCP/HTTP simulator, loads the rounded-plate example, and does not save settings. The simulator supports upload, verification, motion, hold/restore, sparse status fields, auto reports, overrides, absolute/relative jogs, work offsets, reset-during-motion alarms, and connection drops. Automated tests cover controller interactions against it; this is separate from testing a physical cutter.

The GUI smoke check additionally needs Xvfb, xdotool, xclip, zenity, Python Pillow and python-xlib on the development machine:

```sh
xvfb-run -a -s '-screen 0 1280x900x24' python3 scripts/ui_smoke.py
```
