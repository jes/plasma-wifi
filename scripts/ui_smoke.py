#!/usr/bin/env python3
"""Exercise the actual static desktop binary using X11 input and a loopback simulator."""
import json
import os
from pathlib import Path
import re
import subprocess
import time
import urllib.request

from PIL import Image
from Xlib import X, display

ROOT = Path(__file__).resolve().parent.parent
BINARY = ROOT / "dist/plasma-wifi"


def command(*args):
    return subprocess.check_output(["xdotool", *map(str, args)], text=True).strip()


process = subprocess.Popen([str(BINARY), "--demo"], stderr=subprocess.PIPE, text=True)
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

    wait(lambda s: "G4P0.5" in s["file"], "Automatic upload did not finish")
    time.sleep(1.0)  # let verification and UI snapshots settle
    screenshot("idle.png")

    # Editing the DRO changes the work offset, never physical position.
    before = state()
    click(110, 113)
    command("key", "ctrl+a")
    command("type", "--clearmodifiers", "42")
    command("key", "Return")
    changed = wait(lambda s: abs(s["x"] - 42) < 0.001, "DRO Enter did not apply")
    assert abs(changed["mpos_x"] - before["mpos_x"]) < 0.001

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
    assert state()["state"] == "Hold:0", "Motion resumed before the pierce delay"
    wait(lambda s: s["state"] == "Run", "Motion did not resume after pierce delay")
    click(207, 223)  # Stop
    wait(lambda s: s["state"] == "Idle" and not s["torch"], "Stop did not abort and extinguish")

    # The same layout should remain usable on a smaller laptop display.
    command("windowsize", window, 1024, 768)
    time.sleep(0.5)
    screenshot("small-display.png")
    assert process.poll() is None
    print("GUI smoke checks passed: upload, DRO, keyboard focus/release, Run/Pause/Resume/Stop, resize")
    print("Screenshots:", ROOT / "dist")
finally:
    process.terminate()
    process.wait(timeout=3)
