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
BINARY = ROOT / "dist/plasma-wifi"


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
        d.close()
        return target

    uploaded = wait(lambda s: "G1X80Y10F1200" in s["file"], "Automatic upload did not finish")
    assert "G4" not in uploaded["file"], "Upload added a dwell absent from CAM output"
    time.sleep(1.0)  # let verification and UI snapshots settle
    screenshot("idle.png")
    d=display.Display()
    w=d.create_resource_object("window", int(window))
    icon=w.get_full_property(d.intern_atom("_NET_WM_ICON"), X.AnyPropertyType)
    assert icon is not None and len(icon.value)==32*32+64*64+4
    assert icon.value[0:2].tolist()==[32,32]
    d.close()

    # Editing the DRO changes the work offset, never physical position.
    before = state()
    click(110, 113)
    command("key", "ctrl+a")
    command("type", "--clearmodifiers", "42")
    command("key", "Return")
    changed = wait(lambda s: abs(s["x"] - 42) < 0.001, "DRO Enter did not apply")
    assert abs(changed["mpos_x"] - before["mpos_x"]) < 0.001

    time.sleep(0.25)  # the UI must receive the new offset before the next click
    click(110, 113)
    before = state()
    command("keydown", "Right")
    time.sleep(0.35)
    command("keyup", "Right")
    assert abs(state()["mpos_x"] - before["mpos_x"]) < 0.001, "Arrow jogged from text field"

    click(207, 113)  # /2
    wait(lambda s: abs(s["x"] - 21) < 0.001, "DRO /2 did not halve")
    click(25, 113)  # X0
    wait(lambda s: abs(s["x"]) < 0.001, "X0 did not zero")

    click(450, 400)  # clear text focus
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

    click(45, 223)  # Run
    wait(lambda s: s["state"] == "Run" and s["torch"], "Run did not ignite torch")
    time.sleep(1.0)
    screenshot("running.png")
    click(126, 223)  # Pause
    wait(lambda s: s["state"] == "Hold:0" and not s["torch"], "Pause did not extinguish")
    time.sleep(0.3)
    frozen = state()
    command("keydown", "Right")
    time.sleep(0.25)
    command("keyup", "Right")
    assert state()["mpos_x"] == frozen["mpos_x"], "Paused job allowed jogging"
    screenshot("paused.png")
    click(126, 223)  # Resume
    wait(lambda s: s["torch"], "Resume did not restore torch")
    wait(lambda s: s["state"] == "Run", "Motion did not resume after torch restoration")
    click(207, 223)  # Stop
    wait(lambda s: s["state"] == "Idle" and not s["torch"], "Stop did not abort and extinguish")

    assert state()["alarm"] is None, "Normal Stop raised a reset-during-motion alarm"
    time.sleep(1.0)  # reset handshake and verification

    # Console text entry and history must never trigger arrow-key jogging.
    click(1080, 22)
    time.sleep(0.4)
    screenshot("console.png")
    click(300, 424)
    command("type", "--clearmodifiers", "G10L20P1X12.345")
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
    click(752, 33)  # close the console

    def chooser():
        command("windowfocus", window)
        time.sleep(0.15)
        command("key", "ctrl+o")
        end=time.monotonic()+6
        while time.monotonic()<end:
            result=subprocess.run(["xdotool","search","--onlyvisible","--name","^Open G-code$"],capture_output=True,text=True)
            if result.returncode==0:
                chosen=result.stdout.splitlines()[0]
                time.sleep(0.3)  # GTK publishes its title before mapping/initializing the dialog
                command("windowfocus", chosen)
                return chosen
            time.sleep(0.05)
        screenshot("chooser-failure.png")
        raise AssertionError("System file chooser did not open: "+subprocess.run(["xdotool","search","--name",".*","getwindowname"],capture_output=True,text=True).stdout)

    chooser()
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
    click(100, 688)  # Trace hull at jog speed
    wait(lambda s: s["state"]=="Jog", "Hull trace did not start")
    screenshot("hull-trace.png")
    target=f"$J=G21G90X{start['x']:.3f}Y{start['y']:.3f}F600.0"
    traced=wait(lambda s: target in s["commands"][previous:] and s["state"]=="Idle"
                and abs(s["x"]-start["x"])<0.001 and abs(s["y"]-start["y"])<0.001,
                "Hull trace did not close and return")
    assert not traced["torch"], "Hull trace lit the torch"
    time.sleep(0.3)
    click(100, 688)
    wait(lambda s: s["state"]=="Jog", "Second hull trace did not start")
    time.sleep(0.15)
    click(100, 688)  # Cancel hull trace
    wait(lambda s: s["state"]=="Idle", "Hull cancellation did not stop jogging")
    count=len(state()["commands"])
    time.sleep(0.4)
    assert len(state()["commands"])==count, "Hull trace continued after cancellation"

    # The same layout should remain usable on a smaller laptop display.
    command("windowsize", window, 1024, 768)
    time.sleep(0.5)
    screenshot("small-display.png")
    assert process.poll() is None
    print("GUI smoke checks passed: native picker/cancel/upload, icon, console/history, DRO, keyboard focus/release, Run/Pause/Resume/Stop, hull/return/cancel, resize")
    print("Screenshots:", ROOT / "dist")
finally:
    if process.poll() is None:
        process.terminate()
        process.wait(timeout=3)
