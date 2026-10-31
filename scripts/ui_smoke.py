#!/usr/bin/env python3
"""Exercise the actual static desktop binary using X11 input and a loopback simulator."""
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time
import urllib.request

from PIL import Image
from Xlib import X, display

ROOT = Path(__file__).resolve().parent.parent
BINARY = Path(os.environ.get("PLASMA_WIFI_BINARY", ROOT / "dist/plasma-wifi"))


def command(*args):
    return subprocess.check_output(["xdotool", *map(str, args)], text=True).strip()


# Keep the picker inside this test display; force the native helper fallback.
env = dict(os.environ, DBUS_SESSION_BUS_ADDRESS="unix:path=/tmp/plasma-wifi-no-session-bus")
process = subprocess.Popen([str(BINARY), "--demo"], stderr=subprocess.PIPE, text=True, env=env)
try:
    line = process.stderr.readline()
    match = re.search(r"HTTP (http://127\.0\.0\.1:\d+)", line)
    assert match, line
    base = match[1]

    def state():
        with urllib.request.urlopen(base + "/sim/status", timeout=2) as response:
            return json.load(response)

    def wait(condition, message, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            value = state()
            if condition(value):
                return value
            assert process.poll() is None, process.stderr.read()
            time.sleep(0.04)
        screenshot("failure.png")
        click(950, 22)
        screenshot("failure-log.png")
        raise AssertionError(f"{message}: {state()}")

    window = command("search", "--sync", "--name", "^Plasma WiFi$").splitlines()[0]
    command("windowfocus", window)

    def click(x, y):
        command("mousemove", "--window", window, x, y, "click", 1)
        time.sleep(0.12)

    def screenshot(name):
        d = display.Display()
        w = d.create_resource_object("window", int(window))
        geometry = w.get_geometry()
        raw = w.get_image(0, 0, geometry.width, geometry.height, X.ZPixmap, 0xFFFFFFFF)
        image = Image.frombytes("RGB", (geometry.width, geometry.height), raw.data, "raw", "BGRX")
        target = ROOT / "dist" / name
        image.save(target)
        if name == "failure.png":
            screen = d.screen()
            raw = screen.root.get_image(0, 0, screen.width_in_pixels, screen.height_in_pixels, X.ZPixmap, 0xFFFFFFFF)
            Image.frombytes("RGB", (screen.width_in_pixels, screen.height_in_pixels), raw.data, "raw", "BGRX").save(ROOT / "dist" / "failure-desktop.png")
        d.close()
        return target

    uploaded = wait(lambda s: "G1X80Y10F1200" in s["file"], "Automatic upload did not finish")
    assert "G4" not in uploaded["file"], "Upload added a dwell absent from CAM output"
    time.sleep(1.0)  # let verification and UI snapshots settle
    screenshot("default-display.png")
    command("windowsize", window, 1180, 1000)
    time.sleep(0.5)
    screenshot("idle.png")
    normal_text = Image.open(ROOT / "dist" / "idle.png").copy()

    def control_scroll(x, y, button, count):
        command("mousemove", "--window", window, x, y)
        command("keydown", "Control_L")
        command("click", "--repeat", count, "--delay", 80, button)
        command("keyup", "Control_L")
        time.sleep(0.4)

    def heading_height(picture):
        rows = [y for y in range(5, 42) if any(
            (lambda rgb: min(rgb) > 180 and max(rgb) - min(rgb) < 40)(picture.getpixel((x, y)))
            for x in range(5, 80))]
        return max(rows) - min(rows) + 1

    # Original is selected by default. Ctrl+wheel over the header scales the
    # whole interface, including fixed-size labels and coordinate hit targets.
    assert any(normal_text.getpixel((x, y)) == (7, 78, 173) for y in range(550, 650) for x in range(510, 560)), "Original G-code was not selected by default"
    def has_color(picture, box, color):
        pixels = picture.load()
        return any(max(abs(a - b) for a, b in zip(pixels[x, y], color)) <= 5
                   for y in range(box[1], box[3]) for x in range(box[0], box[2]))

    code_box = (415, 615, 1150, 955)
    for color in [(245, 247, 250), (0, 229, 255), (255, 224, 112), (255, 151, 145), (196, 170, 255), (150, 157, 167)]:
        assert has_color(normal_text, code_box, color), f"Original syntax color missing: {color}"
    assert has_color(normal_text, (360, 615, 410, 955), (119, 130, 145)), "Original line numbers were not subdued"
    click(452, 597)  # Uploaded
    uploaded_view = Image.open(screenshot("uploaded-syntax.png")).copy()
    for color in [(245, 247, 250), (0, 229, 255), (255, 224, 112), (255, 151, 145), (196, 170, 255)]:
        assert has_color(uploaded_view, code_box, color), f"Uploaded syntax color missing: {color}"
    click(539, 597)  # Original
    control_scroll(30, 30, 4, 3)
    enlarged_text = Image.open(screenshot("text-zoom.png")).copy()
    assert heading_height(enlarged_text) > heading_height(normal_text), "Ctrl+scroll did not enlarge UI labels"
    before_zoom_edit = state()
    click(180, 176)  # X coordinate field at approximately 135% UI scale.
    command("key", "ctrl+a")
    command("type", "--clearmodifiers", "42")
    command("key", "Return")
    wait(lambda s: abs(s["x"] - 42) < 0.001, "Pointer hit targets did not follow UI zoom")
    assert state()["mpos_x"] == before_zoom_edit["mpos_x"], "Editing a zoomed DRO moved the machine"
    time.sleep(0.2)
    click(42, 176)  # X0 at the enlarged scale.
    wait(lambda s: abs(s["x"]) < 0.001, "Zoomed X0 button did not respond")
    control_scroll(30, 30, 5, 3)
    d=display.Display()
    w=d.create_resource_object("window", int(window))
    icon=w.get_full_property(d.intern_atom("_NET_WM_ICON"), X.AnyPropertyType)
    assert icon is not None and len(icon.value)==32*32+64*64+4
    assert icon.value[0:2].tolist()==[32,32]
    d.close()

    # Editing the DRO changes the work offset, never physical position.
    before = state()
    click(130, 130)
    command("key", "g", "m", "x", "y", "f", "i", "r", "p", "s", "o")  # Shortcuts leave a focused input alone.
    command("key", "ctrl+a")
    command("type", "--clearmodifiers", "42")
    command("key", "Return")
    changed = wait(lambda s: abs(s["x"] - 42) < 0.001, "DRO Enter did not apply")
    assert abs(changed["mpos_x"] - before["mpos_x"]) < 0.001

    time.sleep(0.25)  # the UI must receive the new offset before the next click
    click(130, 130)
    before = state()
    command("keydown", "Right")
    time.sleep(0.35)
    command("keyup", "Right")
    assert abs(state()["mpos_x"] - before["mpos_x"]) < 0.001, "Arrow jogged from text field"

    click(305, 130)  # /2
    wait(lambda s: abs(s["x"] - 21) < 0.001, "DRO /2 did not halve")
    click(30, 130)  # X0
    wait(lambda s: abs(s["x"]) < 0.001, "X0 did not zero")

    # Keyboard field shortcuts select the current value and commit on Enter.
    click(450, 400)
    physical = state()

    def edit_shortcut(key, text):
        command("key", key)
        time.sleep(0.12)
        command("type", "--clearmodifiers", "--", text)
        command("key", "Return")
        time.sleep(0.15)

    edit_shortcut("x", "20")
    wait(lambda s: abs(s["x"] - 20) < 0.001, "X shortcut did not edit the coordinate")
    edit_shortcut("x", "/2")
    wait(lambda s: abs(s["x"] - 10) < 0.001, "X /2 expression did not halve")
    edit_shortcut("y", "-8")
    wait(lambda s: abs(s["y"] + 8) < 0.001, "Y shortcut did not edit the coordinate")
    edit_shortcut("y", "/2")
    wait(lambda s: abs(s["y"] + 4) < 0.001, "Y /2 expression did not halve")
    assert state()["mpos_x"] == physical["mpos_x"] and state()["mpos_y"] == physical["mpos_y"], "Coordinate shortcuts moved the machine"
    edit_shortcut("x", "0")
    edit_shortcut("y", "0")
    edit_shortcut("f", "400")
    edit_shortcut("i", "0.25")
    edit_shortcut("f", "/2")  # Only coordinate fields support /2.
    edit_shortcut("i", "/2")
    before = state()
    command("key", "Right")
    changed = wait(lambda s: s["state"] == "Idle" and abs(s["mpos_x"] - before["mpos_x"] - 0.25) < 0.001,
                   "I shortcut did not set the increment, or accepted /2")
    assert [c for c in changed["commands"] if c.startswith("$J=")][-1].endswith("F400.0"), "F shortcut did not set jog feed, or accepted /2"
    edit_shortcut("f", "600")
    edit_shortcut("i", "1")

    click(450, 400)  # clear text focus
    before = state()
    count = len([c for c in before["commands"] if c.startswith("$J=")])
    command("key", "Right")
    tapped = wait(lambda s: s["state"] == "Idle" and abs(s["mpos_x"] - before["mpos_x"] - 1) < 0.001,
                  "Quick arrow tap did not move one increment")
    assert len([c for c in tapped["commands"] if c.startswith("$J=")]) == count + 1
    before = state()
    click(165, 693)  # Y+ must also work for a quick mouse click.
    wait(lambda s: s["state"] == "Idle" and abs(s["mpos_y"] - before["mpos_y"] - 1) < 0.001,
         "Mouse tap did not move one increment")
    click(450, 400)
    before = state()
    command("keydown", "Right")
    time.sleep(0.45)
    command("keyup", "Right")
    wait(lambda s: s["state"] == "Idle", "Jog did not cancel on key release")
    assert state()["mpos_x"] > before["mpos_x"], "Arrow key did not jog"

    # Loss of focus must cancel jogging even without receiving a key release.
    command("keydown", "Left")
    wait(lambda s: s["state"] == "Jog", "Second jog did not start")
    d = display.Display()
    d.screen().root.set_input_focus(X.RevertToParent, X.CurrentTime)
    d.sync()
    wait(lambda s: s["state"] == "Idle", "Jog did not cancel on focus loss")
    command("keyup", "Left")
    command("windowfocus", window)
    time.sleep(0.3)  # receive the cancellation status before clicking Run

    click(450, 400)
    before = state()
    command("key", "shift+Right")
    rapid = wait(lambda s: s["state"] == "Idle" and abs(s["mpos_x"] - before["mpos_x"] - 1) < 0.001,
                 "Shift+arrow did not jog one increment")
    assert any(c.startswith("$J=") and c.endswith("F2400.0") for c in rapid["commands"][len(before["commands"]):]), "Shift did not multiply keyboard jog feed"

    # Shift also changes speed during a held mouse jog, without pointer movement.
    baseline = len(state()["commands"])
    command("mousemove", "--window", window, 270, 739, "mousedown", 1)
    wait(lambda s: any(c.startswith("$J=") and c.endswith("F600.0") for c in s["commands"][baseline:]), "Held mouse jog did not start")
    baseline = len(state()["commands"])
    command("keydown", "Shift_L")
    wait(lambda s: any(c.startswith("$J=") and c.endswith("F2400.0") for c in s["commands"][baseline:]), "Shift did not boost held mouse jog")
    baseline = len(state()["commands"])
    command("keyup", "Shift_L")
    wait(lambda s: any(c.startswith("$J=") and c.endswith("F600.0") for c in s["commands"][baseline:]), "Releasing Shift did not slow held mouse jog")
    command("mouseup", 1)
    wait(lambda s: s["state"] == "Idle", "Mouse release did not finish jogging")
    time.sleep(0.3)  # Receive Idle and enable Run in the UI before clicking.

    click(55, 298)  # Run
    wait(lambda s: s["state"] == "Run" and s["torch"], "Run did not ignite torch")
    time.sleep(1.0)
    screenshot("running.png")
    click(165, 298)  # Pause
    wait(lambda s: s["state"] == "Hold:0" and not s["torch"], "Pause did not extinguish")
    time.sleep(0.3)
    frozen = state()
    command("keydown", "Right")
    time.sleep(0.25)
    command("keyup", "Right")
    assert state()["mpos_x"] == frozen["mpos_x"], "Paused job allowed jogging"
    screenshot("paused.png")
    click(165, 298)  # Resume
    wait(lambda s: s["torch"], "Resume did not restore torch")
    wait(lambda s: s["state"] == "Run", "Motion did not resume after torch restoration")
    click(275, 298)  # Stop
    wait(lambda s: s["state"] == "Idle" and not s["torch"], "Stop did not abort and extinguish")

    assert state()["alarm"] is None, "Normal Stop raised a reset-during-motion alarm"
    time.sleep(1.0)  # reset handshake and verification

    # R/P/S use the same run, pause, resume and stop gates as the buttons.
    click(450, 400)
    command("key", "r")
    wait(lambda s: s["state"] == "Run" and s["torch"], "R did not run")
    time.sleep(0.3)  # let the UI receive the new controller state
    command("keydown", "p")
    wait(lambda s: s["state"] == "Hold:0" and not s["torch"], "P did not pause")
    end = time.monotonic() + 1.2
    while time.monotonic() < end:
        held = state()
        assert held["state"] == "Hold:0" and not held["torch"], "Holding P toggled pause/resume"
        time.sleep(0.025)
    command("keyup", "p")
    time.sleep(0.3)
    command("key", "r")
    wait(lambda s: s["state"] == "Run" and s["torch"], "R did not resume")
    time.sleep(0.3)
    command("key", "p")
    wait(lambda s: s["state"] == "Hold:0" and not s["torch"], "Second P did not pause")
    time.sleep(0.3)
    command("key", "p")
    wait(lambda s: s["state"] == "Run" and s["torch"], "P did not resume")
    command("key", "s")
    wait(lambda s: s["state"] == "Idle" and not s["torch"], "S did not stop")
    time.sleep(1.0)

    # Console text entry and history must never trigger arrow-key jogging.
    click(450, 400)
    command("key", "g")
    time.sleep(0.4)
    screenshot("console.png")
    console_normal = Image.open(ROOT / "dist" / "console.png").copy()
    control_scroll(300, 250, 4, 2)
    console_enlarged = Image.open(screenshot("console-text-zoom.png")).copy()
    assert console_normal.crop((20, 80, 700, 400)).tobytes() != console_enlarged.crop((20, 80, 700, 400)).tobytes(), "Ctrl+scroll did not resize console text"
    control_scroll(300, 250, 5, 2)
    command("type", "--clearmodifiers", "10L20P1X12.345")
    command("key", "Return")
    wait(lambda s: abs(s["x"]-12.345)<0.001, "Console command did not apply")
    time.sleep(0.25)
    before=state()
    count=before["commands"].count("G10L20P1X12.345")
    command("key", "Up", "Return")
    wait(lambda s: s["commands"].count("G10L20P1X12.345")>count, "Console history did not recall")
    assert state()["mpos_x"]==before["mpos_x"], "Console history arrow jogged"
    count=state()["commands"].count("$I")
    command("type", "--clearmodifiers", "$I")
    command("key", "Up", "Down", "Return")
    wait(lambda s: s["commands"].count("$I")>count, "Console history did not restore draft")
    count=state()["commands"].count("M5")
    command("type", "--clearmodifiers", "M5")  # Already focused: ordinary text input.
    command("key", "Return")
    wait(lambda s: s["commands"].count("M5")>count, "M shortcut interfered with console typing")
    before = state()
    command("type", "--clearmodifiers", "G10L20P1X999")
    command("key", "Escape")  # Close while editing, without submitting the draft.
    time.sleep(0.15)
    command("key", "Right")
    wait(lambda s: s["state"] == "Idle" and abs(s["mpos_x"] - before["mpos_x"] - 1) < 0.001,
         "Escape did not close the console and release input focus")
    assert "G10L20P1X999" not in state()["commands"], "Escape submitted the console draft"
    click(450, 400)
    count=state()["commands"].count("M5")
    command("key", "shift+m")
    time.sleep(0.2)
    command("type", "--clearmodifiers", "5")
    command("key", "Return")
    wait(lambda s: s["commands"].count("M5")>count, "M shortcut did not focus and prefill the console")
    command("key", "Escape")

    def chooser(title="Open G-code", open_action=None):
        command("windowfocus", window)
        time.sleep(0.15)
        if open_action is None:
            command("key", "ctrl+o")
        else:
            open_action()
        end=time.monotonic()+6
        while time.monotonic()<end:
            result=subprocess.run(["xdotool","search","--onlyvisible","--name",f"^{re.escape(title)}$"],capture_output=True,text=True)
            if result.returncode==0:
                chosen=result.stdout.splitlines()[0]
                time.sleep(0.3)  # GTK publishes its title before mapping/initializing the dialog
                command("windowfocus", chosen)
                return chosen
            time.sleep(0.05)
        screenshot("chooser-failure.png")
        raise AssertionError("System file chooser did not open: "+subprocess.run(["xdotool","search","--name",".*","getwindowname"],capture_output=True,text=True).stdout)

    chooser(open_action=lambda: command("key", "o"))
    command("key", "Escape")
    time.sleep(1.0)
    assert state()["file"]==before["file"], "Cancelling picker changed the job"
    with tempfile.TemporaryDirectory(prefix="plasma-wifi-ui-") as folder:
        source=Path(folder)/"hull trace.nc"
        source.write_text("G021G090\nG00X10.00001Y0.00001\nM3S1\nG01X10Y10F600\nX5Y5\nX0Y10\nM5\nM30\n")
        chooser()
        command("key", "ctrl+l")
        subprocess.run(["xclip", "-selection", "clipboard"], input=str(source), text=True, check=True, stderr=subprocess.DEVNULL)
        command("key", "ctrl+v")
        time.sleep(0.2)
        command("key", "Return")
        wait(lambda s: "G0X10Y0" in s["file"] and "G1X10Y10F600" in s["file"], "Native picker did not load and upload file")
    command("windowfocus", window)
    time.sleep(0.6)
    start=state()
    previous=len(start["commands"])
    click(135, 900)  # Trace hull at jog speed
    wait(lambda s: s["state"]=="Jog", "Hull trace did not start")
    screenshot("hull-trace.png")
    target=f"$J=G21G90X{start['x']:.3f}Y{start['y']:.3f}F600.0"
    traced=wait(lambda s: target in s["commands"][previous:] and s["state"]=="Idle"
                and abs(s["x"]-start["x"])<0.001 and abs(s["y"]-start["y"])<0.001,
                "Hull trace did not close and return")
    assert not traced["torch"], "Hull trace lit the torch"
    time.sleep(0.3)
    click(135, 900)
    wait(lambda s: s["state"]=="Jog", "Second hull trace did not start")
    baseline = len(state()["commands"])
    command("keydown", "Shift_L")
    wait(lambda s: any(c.startswith("$J=") and c.endswith("F2400.0") for c in s["commands"][baseline:]),
         "Holding Shift did not boost the active hull trace")
    baseline = len(state()["commands"])
    command("keyup", "Shift_L")
    wait(lambda s: any(c.startswith("$J=") and c.endswith("F600.0") for c in s["commands"][baseline:]),
         "Releasing Shift did not slow the active hull trace")
    click(135, 900)  # Cancel hull trace
    wait(lambda s: s["state"]=="Idle", "Hull cancellation did not stop jogging")
    count=len(state()["commands"])
    time.sleep(0.4)
    assert len(state()["commands"])==count, "Hull trace continued after cancellation"

    # Download config, upload an exact replacement, and save the changed bytes.
    click(898, 22)  # Files
    time.sleep(0.4)
    screenshot("controller-files.png")
    before = state()
    command("key", "Right")
    time.sleep(0.15)
    assert state()["mpos_x"] == before["mpos_x"], "Files dialog allowed keyboard jogging"
    with urllib.request.urlopen(base + "/localfs/config.yaml", timeout=2) as response:
        original_config = response.read()
    with tempfile.TemporaryDirectory(prefix="plasma-wifi-files-") as folder:
        folder = Path(folder)

        def select_path(path):
            command("key", "ctrl+l")
            command("key", "ctrl+a")
            subprocess.run(["xclip", "-selection", "clipboard"], input=str(path), text=True, check=True, stderr=subprocess.DEVNULL)
            command("key", "ctrl+v")
            time.sleep(0.2)
            command("key", "Return")

        backup = folder / "original config.yaml"
        chooser("Download controller file", lambda: click(430, 225))
        select_path(backup)
        wait(lambda s: backup.exists() and backup.read_bytes() == original_config,
             "Save dialog did not download config.yaml unchanged")
        command("windowfocus", window)
        time.sleep(0.4)

        source = folder / "config.yaml"
        changed_config = original_config + b"\n# GUI upload round trip\n"
        source.write_bytes(changed_config)
        chooser("Upload controller file", lambda: click(155, 101))
        select_path(source)

        def config_uploaded(_):
            with urllib.request.urlopen(base + "/localfs/config.yaml", timeout=2) as response:
                return response.read() == changed_config

        wait(config_uploaded, "Upload dialog did not replace config.yaml")
        command("windowfocus", window)
        time.sleep(0.5)
        assert state()["file"] == before["file"], "Config upload replaced the G-code job"
        changed_backup = folder / "changed backup.yaml"
        chooser("Download controller file", lambda: click(430, 225))
        select_path(changed_backup)
        wait(lambda s: changed_backup.exists() and changed_backup.read_bytes() == changed_config,
             "Uploaded config did not download through Save As")
        command("windowfocus", window)
        time.sleep(0.3)
        chooser("Upload controller file", lambda: click(155, 101))
        command("key", "Escape")
        time.sleep(0.4)
        assert config_uploaded(None), "Cancelling upload changed config.yaml"
    command("windowfocus", window)
    click(573, 33)  # close Controller files

    # The same layout should remain usable on a smaller laptop display.
    command("windowsize", window, 1024, 768)
    time.sleep(0.5)
    fixed_before = Image.open(screenshot("small-display.png")).crop((0, 80, 344, 460)).tobytes()
    command("mousemove", "--window", window, 180, 680)
    command("click", "--repeat", 18, "--delay", 30, 5)
    time.sleep(0.3)
    scrolled = Image.open(screenshot("small-scrolled.png"))
    assert scrolled.crop((0, 80, 344, 460)).tobytes() == fixed_before, "Scrolling hid or moved the DRO / Run / Pause / Stop"
    assert process.poll() is None
    print("GUI smoke checks passed: native picker/cancel/upload, config upload/download/save/cancel, icon, G/M console/Escape/history/input focus, R/P/S run/pause/resume/stop, O/Ctrl+O open, syntax colors/line numbers, X/Y and /2, F/I numeric edits, jog tap/hold/Shift boost/focus, Run/Pause/Resume/Stop, hull/Shift boost/return/cancel, resize")
    print("Screenshots:", ROOT / "dist")
finally:
    if process.poll() is None:
        process.terminate()
        process.wait(timeout=3)
