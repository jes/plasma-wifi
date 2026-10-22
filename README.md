# Plasma WiFi

A Linux desktop controller for an XY plasma cutter running FluidNC 4.1.1. Jobs are uploaded to LocalFS as `plasma.nc`, verified by size and a complete download comparison, then executed on the controller. Motion does not depend on streaming from the laptop.

## Run

Copy `dist/plasma-wifi` to an x86-64 Linux laptop and run it:

```sh
chmod +x plasma-wifi
./plasma-wifi
```

The executable is statically linked with musl. It needs an X11 display, or XWayland on a Wayland desktop. It does not load host glibc, libX11, GTK, Qt, or OpenGL. Rendering and bundled fonts are built into the binary.

The default controller is `192.168.0.1:23`, with HTTP on port 80. Change these in Settings or use:

```sh
./plasma-wifi --controller 192.168.0.1:23 --http http://192.168.0.1 job.nc
```

Install the supplied `config.yaml` through the FluidNC WebUI before using active-line tracking: it adds `use_line_numbers: true` and retains the existing motor configuration, including the unused Z pins. The app does not install controller configuration automatically.

## Use

- Open a G-code file with **Open G-code** or **Ctrl+O**. Processing, uploading, and verification happen automatically. The header shows progress and whether the file is verified. **Run** starts it explicitly.
- Jog using held on-screen buttons or arrow keys. The speed is in mm/min. Keyboard jogging is suppressed in text fields and dialogs, and ends on key release or focus loss. Short jog segments require a continuing UI heartbeat.
- **X0**, **Y0**, and **/2** change work coordinates without moving. Click a DRO number, edit it, and press **Enter** to apply. The app uses `G54` and `G10 L20 P1` for work offsets; displayed coordinates are always millimetres.
- Feed controls adjust the controller's realtime feed override. Pierce dwell is unaffected.
- Set pierce delay in **Settings**. Applying a change regenerates and uploads the loaded job. The original file is never edited.
- **Pause** holds motion, then extinguishes a lit torch. **Resume** restores it only if it was lit, waits the configured delay while motion remains held, then resumes. **Stop** soft-resets the controller, aborting motion and extinguishing the torch. A later Run starts the file again from the beginning.
- The G-code panel switches between original and uploaded lines. Inserted pierce dwells are marked. Auto-scroll follows the active motion line; FluidNC may omit line numbers during a dwell or between planner blocks, so the most recent known line remains highlighted.
- The preview distinguishes rapid moves, torch-on cuts, the active segment, and completed segments. Drag to pan, wheel to zoom, and **Fit** to reset the view.

Files cannot be replaced during running or paused jobs. Run is gated on verified contents and fresh idle controller status. Reconnection queries state without replaying Run, coordinate writes, or jogs. Elapsed time is marked uncertain after a connection gap because an unobserved pause or override change cannot be reconstructed exactly.

## G-code and timing

The processor handles linear moves, XY `G2/G3` arcs with I/J or signed R, full circles with explicit X/Y endpoints, helical Z coordinates, arc P rotations, G20/G21 units, G90/G91 positioning, G91.1 incremental arc centers, G94 feed, G4 dwell, G53 machine moves, G54, common FreeCAD Grbl preamble words, M3/M5, and M2/M30. Source numerical precision is preserved. Combined ignition/motion blocks are split so the inserted pierce dwell happens before motion. Original line numbers are replaced and mapped to their source lines.

The UI has no Z controls. Existing Z commands are retained and explicitly flagged, so a milling post's retract moves are never silently deleted. Use an XY/plasma CAM setup for this machine. Unsupported coordinate systems, planes, expressions, probing, canned cycles, and other unmodelled commands produce a source-line error and are not uploaded. FreeCAD's Grbl post normally disables tool-length compensation; leave it disabled. A representative Grbl preamble is covered by the parser tests.

Timing separates feed moves, axis-limited rapid moves, and fixed dwells. Cumulative times and projection of the reported position onto the active toolpath locate progress. Historical feed overrides normalize completed work before calibrating the motion-time coefficient; the current override scales future feed work. Dwells and additional Resume pierces are accounted for separately, and paused time is excluded. Estimates remain approximate because of acceleration, planner blending, polling latency, and unsupported hardware feedback such as arc-established sensing.

Pause/resume uses FluidNC's realtime feed hold (`!`), spindle-stop override (`0x9e`) while held, and cycle start (`~`). Torch state is confirmed from accessory reports before the app proceeds. The Resume delay starts after the restored torch is reported on. These semantics were checked against the [4.1.1 suspend and spindle-management implementation](https://github.com/bdring/FluidNC/blob/v4.1.1/FluidNC/src/Protocol.cpp) and [parking spindle restoration](https://github.com/bdring/FluidNC/blob/v4.1.1/FluidNC/src/Parking.cpp). The original pierce dwell is injected as `G4 P<seconds>` into the LocalFS job.

## Build and check

```sh
sudo apt install musl-tools
rustup target add x86_64-unknown-linux-musl
./scripts/build.sh
cargo test --locked --release --target x86_64-unknown-linux-musl
```

Cargo fetches the Rust dependencies. `Cargo.lock` pins them. Settings are saved under `$XDG_CONFIG_HOME/plasma-wifi/settings.json`, or `~/.config/plasma-wifi/settings.json`.

To explore without hardware:

```sh
./dist/plasma-wifi --demo
```

Demo starts a loopback-only TCP/HTTP simulator, loads the rounded-plate example, and does not save settings. The simulator supports upload, verification, motion, hold/restore, overrides, jogs, work offsets, and connection drops. Automated tests cover controller interactions against it; this is separate from testing a physical cutter.

The GUI smoke check additionally needs Xvfb, xdotool, Python Pillow and python-xlib on the development machine:

```sh
xvfb-run -a -s '-screen 0 1280x900x24' python3 scripts/ui_smoke.py
```
