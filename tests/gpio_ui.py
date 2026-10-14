#!/usr/bin/env python3
"""End-to-end check of the web UI path: flash the GPIO demo firmware, then use
only the UI's HTTP API (the same calls the browser makes) to watch pins and
drive inputs, and check the firmware reacts.

  python3 tests/gpio_ui.py [--emulator build/esp32sim]

Exits 77 (skipped) when the ROM image or esptool is missing.
"""
import argparse
import base64
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
FW = ROOT / "firmware" / "gpio-demo" / "prebuilt"
ROM = ROOT / "roms" / "esp32c3_rev3_rom.elf"
TCP, UI = 5572, 18080
BASE = f"http://127.0.0.1:{UI}"


class Events(threading.Thread):
    """Reads the /events stream, keeps the latest state and all serial text."""

    def __init__(self):
        super().__init__(daemon=True)
        self.state, self.serial, self.lock = None, "", threading.RLock()

    def run(self):
        with urllib.request.urlopen(BASE + "/events", timeout=60) as r:
            event = None
            for raw in r:
                line = raw.decode().rstrip("\n")
                if line.startswith("event: "):
                    event = line[7:]
                elif line.startswith("data: "):
                    with self.lock:
                        if event == "state":
                            self.state = json.loads(line[6:])
                        elif event == "serial":
                            self.serial += base64.b64decode(line[6:]).decode(errors="replace")

    def pin(self, gpio):
        with self.lock:
            return next((p for p in (self.state or {}).get("pins", []) if p["gpio"] == gpio), None)

    def wait(self, what, cond, timeout=10):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if cond():
                    return True
            time.sleep(0.05)
        return False


def post(path, body):
    req = urllib.request.Request(BASE + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=5).read()


def main():
    ap = argparse.ArgumentParser()
    exe = "esp32sim.exe" if os.name == "nt" else "esp32sim"
    ap.add_argument("--emulator", default=str(ROOT / "build" / exe))
    args = ap.parse_args()
    if not ROM.exists():
        print(f"skip: {ROM} missing (python3 tools/fetch_roms.py)")
        return 77
    try:
        import esptool  # noqa: F401
    except ImportError:
        print("skip: esptool not installed")
        return 77

    failed = 0

    def check(ok, what):
        nonlocal failed
        failed += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {what}")

    with tempfile.TemporaryDirectory() as tmp:
        flash = os.path.join(tmp, "flash.bin")
        emu = subprocess.Popen([args.emulator, "--flash", flash, "--tcp", str(TCP), "--ui", str(UI),
                                "--boot", "download", "--no-console", "--no-gpio-log", "--rom", str(ROM)],
                               cwd=tmp, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(0.5)
            r = subprocess.run([sys.executable, "-m", "esptool", "--port", f"socket://localhost:{TCP}",
                                "--before", "no-reset", "--after", "no-reset", "write-flash",
                                "0x0", str(FW / "bootloader.bin"), "0x8000", str(FW / "partition-table.bin"),
                                "0x10000", str(FW / "gpio_demo.bin")], capture_output=True, text=True, timeout=120)
            check(r.stdout.count("Hash of data verified") == 3, "GPIO demo flashed")
            if failed:
                print(r.stdout + r.stderr)
                return 1

            catalog = json.loads(urllib.request.urlopen(BASE + "/api/catalog", timeout=5).read())
            xiao = next((b for b in catalog["boards"] if b["id"] == "seeed_xiao_esp32c3"), None)
            check(xiao is not None and xiao["layout"], "catalog lists the XIAO ESP32C3 with a board layout")
            page = urllib.request.urlopen(BASE + "/", timeout=5).read()
            check(b"<title>esp32sim</title>" in page, "UI page served")

            ev = Events()
            ev.start()
            post("/api/reset", {"hold": False})  # RESET button: boot the app
            check(ev.wait("app", lambda: "gpio_demo:" in ev.serial, 15), "app booted (serial console)")
            check(ev.wait("out", lambda: (ev.pin(2) or {}).get("out") == 1), "GPIO2 shown as output")
            levels = set()
            ev.wait("blink", lambda: levels.add((ev.pin(10) or {}).get("lvl")) or levels >= {0, 1}, 3)
            check(levels >= {0, 1}, "GPIO10 seen blinking")
            p4 = ev.pin(4) or {}
            check(p4.get("in") == 1 and p4.get("pu") == 1 and p4.get("lvl") == 1, "GPIO4 input with pull-up reads high")
            post("/api/pin", {"gpio": 4, "level": 0})
            check(ev.wait("press", lambda: "button GPIO4 pressed" in ev.serial), "driving GPIO4 low: firmware sees it")
            check(ev.wait("follow", lambda: (ev.pin(3) or {}).get("lvl") == 1), "GPIO3 follows (code drives it high)")
            post("/api/pin", {"gpio": 4, "level": None})
            check(ev.wait("release", lambda: "button GPIO4 released" in ev.serial), "releasing GPIO4: pull-up wins")
            post("/api/pin", {"gpio": 5, "level": 1})
            check(ev.wait("irq", lambda: "GPIO5 changed, now 1" in ev.serial), "GPIO5 edge interrupt fires")
            post("/api/pin", {"gpio": 9, "level": 0})
            check(ev.wait("boot", lambda: "BOOT button pressed" in ev.serial), "BOOT button interrupt fires")
            post("/api/reset", {"hold": True})
            post("/api/reset", {"hold": False})
            check(ev.wait("dl", lambda: (ev.state or {}).get("status") == "download mode"),
                  "RESET with BOOT held enters download mode")
            post("/api/pin", {"gpio": 9, "level": None})
            post("/api/board", {"board": "seeed_xiao_esp32c3"})
            check(ev.wait("board", lambda: (ev.state or {}).get("board") == "seeed_xiao_esp32c3"), "board switch to XIAO")
            if failed:
                print(ev.serial[-2000:])
            return 1 if failed else 0
        finally:
            emu.terminate()
            emu.wait(timeout=5)


if __name__ == "__main__":
    sys.exit(main())
