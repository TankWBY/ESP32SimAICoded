#!/usr/bin/env python3
"""Arduino-ESP32 compatibility suite: runs the prebuilt arduino_compat sketch
(firmware/arduino-compat) on an emulated chip and checks every test it reports.

  python3 tests/arduino_compat.py --chip esp32c3 [--variant uart|usbcdc] [--emulator build/esp32sim]
  python3 tests/arduino_compat.py --coverage   # every emulated chip has its builds?

The sketch is self-checking: it prints "@PASS name" or "@FAIL name detail"
per test. This script plays the outside world for the tests that need one:
it answers "@ASK" lines by typing on the serial port, driving pins through the
web UI's HTTP API and watching pin levels on its event stream.

Each chip runs the sketch in up to two builds: "uart" (Serial on UART0,
Arduino's default for DevKits) and, on chips with a USB-Serial-JTAG port,
"usbcdc" (Serial on its CDC port: "USB CDC On Boot", the default for boards
such as the Seeed XIAO).

A chip counts as emulated when `esp32sim --list-chips` says so. --coverage
fails when such a chip has no prebuilt "uart" sketch, so a newly emulated chip
cannot skip the suite (build it with tools/build_arduino_compat.py).

Exits 77 (skipped) when the chip's ROM image is missing.
"""
import argparse
import base64
import json
import os
import pathlib
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
PREBUILT = ROOT / "firmware" / "arduino-compat" / "prebuilt"
# Builds of the sketch: image file (bootloader, partition table, boot_app0 and
# app, merged at offset 0) and the emulator options it needs.
VARIANTS = {
    "uart": ("arduino_compat.bin", []),
    "usbcdc": ("arduino_compat_usbcdc.bin", ["--usb-console"]),
}

# Every test the sketch runs. A test missing from the report fails the run,
# so a sketch that silently stops early is caught.
EXPECTED = [
    "chip_identity", "heap", "cpp_runtime", "string_and_math", "random", "millis_micros", "cpu_frequency",
    "ticker", "hw_timer", "freertos", "std_thread", "gpio_output", "gpio_input_pulls", "gpio_interrupt_external",
    "gpio_interrupt_loopback", "serial_echo", "serial1_loopback", "preferences", "littlefs", "flash_raw", "crypto",
    "aes", "ledc_pwm", "i2c_eeprom", "spi_transfer", "adc", "restart_persistence",
]
ECHO_LINE = b"the quick brown fox 0123456789\n"
BURST = bytes(ord("A") + i % 26 for i in range(1000)) + b"\n"


def status(line):
    """The STATUS column of a `--list-chips` row (the first of its words)."""
    m = re.search(r"\s(emulated|not yet)\b", line)
    return m and m.group(1)


def emulated_chips(emulator):
    out = subprocess.run([emulator, "--list-chips"], capture_output=True, text=True, timeout=30).stdout
    return [line.split()[0] for line in out.splitlines()[1:] if status(line) == "emulated"]


def rom_for(chip):
    roms = sorted((ROOT / "roms").glob(f"{chip}_rev*_rom.elf"))
    return roms[0] if roms else None


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Ui(threading.Thread):
    """Follows the web UI's /events stream and keeps the latest pin states."""

    def __init__(self, base):
        super().__init__(daemon=True)
        self.base, self.state, self.lock = base, None, threading.Lock()

    def run(self):
        try:
            with urllib.request.urlopen(self.base + "/events", timeout=600) as r:
                event = None
                for raw in r:
                    line = raw.decode().rstrip("\n")
                    if line.startswith("event: "):
                        event = line[7:]
                    elif line.startswith("data: ") and event == "state":
                        with self.lock:
                            self.state = json.loads(line[6:])
        except OSError:
            pass

    def level(self, gpio):
        with self.lock:
            pin = next((p for p in (self.state or {}).get("pins", []) if p["gpio"] == gpio), None)
        return pin and pin.get("lvl")

    def drive(self, gpio, level):
        req = urllib.request.Request(self.base + "/api/pin", json.dumps({"gpio": gpio, "level": level}).encode(),
                                     {"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=5).read()


def answer(ask, sock, ui):
    """Does what an "@ASK" line asks; returns False if it could not."""
    words = ask.split()
    if words[0] == "ECHO":
        sock.sendall(ECHO_LINE)
        return True  # the sketch checks the line itself
    if words[0] == "BURST":
        sock.sendall(BURST)
        return True
    if words[0] == "DRIVE":
        ui.drive(int(words[1]), None if words[2] == "z" else int(words[2]))
        time.sleep(0.1)  # let the state reach the chip
    elif words[0] == "PIN":
        gpio, want = int(words[1]), int(words[2])
        end = time.time() + 5
        while ui.level(gpio) != want and time.time() < end:
            time.sleep(0.02)
        if ui.level(gpio) != want:
            print(f"  harness: GPIO{gpio} is {ui.level(gpio)} from outside, want {want}")
            sock.sendall(b"NO\n")
            return False
    else:
        print(f"  harness: unknown request '{ask}'")
        sock.sendall(b"NO\n")
        return False
    sock.sendall(b"OK\n")
    return True


def run_chip(emulator, chip, variant, verbose):
    image_name, extra = VARIANTS[variant]
    image = PREBUILT / chip / image_name
    if not image.exists():
        print(f"FAIL no Arduino build for {chip}: {image} (tools/build_arduino_compat.py --chip {chip})")
        return 1
    rom = rom_for(chip)
    if not rom:
        print(f"skip: no ROM image for {chip} in roms/ (python3 tools/fetch_roms.py)")
        return 77

    tcp, ui_port = free_port(), free_port()
    with tempfile.TemporaryDirectory() as tmp:
        flash = os.path.join(tmp, "flash.bin")
        with open(flash, "wb") as f:  # the merged image, as esptool would write it
            f.write(image.read_bytes())
        emu = subprocess.Popen([emulator, "--chip", chip, "--flash", flash, "--flash-size", "4MB",
                                "--tcp", str(tcp), "--ui", str(ui_port), "--rom", str(rom),
                                "--boot", "download", "--no-console", "--no-gpio-log", "--i2c-dev", "eeprom@0x50", *extra],
                               cwd=tmp, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                               text=True)
        results, notes, out = {}, [], b""
        try:
            base = f"http://127.0.0.1:{ui_port}"
            sock = None
            for _ in range(100):
                try:
                    sock = socket.create_connection(("127.0.0.1", tcp), timeout=1)
                    urllib.request.urlopen(base + "/api/catalog", timeout=2).read()
                    break
                except OSError:
                    time.sleep(0.1)
            if sock is None:
                print("FAIL emulator did not start")
                return 1
            ui = Ui(base)
            ui.start()
            # Start in download mode, then press RESET: the app boots once the
            # serial and UI connections are up, so no output is missed.
            urllib.request.urlopen(urllib.request.Request(base + "/api/reset", json.dumps({"hold": False}).encode(),
                                                          {"Content-Type": "application/json"}), timeout=5).read()
            sock.settimeout(0.2)
            pending = b""
            deadline = time.time() + 300
            done = False
            while time.time() < deadline and not done:
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    break
                out += data
                pending += data
                while b"\n" in pending:
                    raw, pending = pending.split(b"\n", 1)
                    line = raw.decode(errors="replace").strip()
                    if verbose and line:
                        print("  | " + line)
                    m = re.search(r"@(\w+) ?(.*)", line)
                    if not m:
                        continue
                    tag, rest = m.groups()
                    if tag == "BEGIN":
                        print(f"  sketch: {rest}")
                    elif tag in ("PASS", "FAIL"):
                        name, _, detail = rest.partition(" ")
                        results[name] = (tag == "PASS", detail)
                        print(f"  {'ok  ' if tag == 'PASS' else 'FAIL'} {name}{'  ' + detail if detail else ''}")
                    elif tag == "ASK":
                        answer(rest, sock, ui)
                    elif tag == "NOTE":
                        notes.append(rest)
                    elif tag == "END":
                        done = True
        finally:
            emu.terminate()
            try:
                emu.wait(timeout=5)
            except subprocess.TimeoutExpired:
                emu.kill()

    missing = [t for t in EXPECTED if t not in results]
    unknown = [t for t in results if t not in EXPECTED]
    for t in missing:
        print(f"  FAIL {t}  (not reported)")
    for t in unknown:
        print(f"  note {t} is not in EXPECTED; add it to tests/arduino_compat.py")
    for n in notes:
        print(f"  note {n}")
    failed = [t for t, (ok, _) in results.items() if not ok] + missing
    print(f"{chip} ({variant}): {sum(ok for ok, _ in results.values())} passed, {len(failed)} failed")
    if failed and not verbose:
        print("--- serial tail ---\n" + out.decode(errors="replace")[-3000:])
    return 1 if failed or not done else 0


def main():
    ap = argparse.ArgumentParser()
    exe = "esp32sim.exe" if os.name == "nt" else "esp32sim"
    ap.add_argument("--emulator", default=str(ROOT / "build" / exe))
    ap.add_argument("--chip", help="chip to run on (default: every emulated chip)")
    ap.add_argument("--variant", choices=sorted(VARIANTS), help="sketch build (default: all)")
    ap.add_argument("--coverage", action="store_true", help="only check every emulated chip has an Arduino build")
    ap.add_argument("-v", "--verbose", action="store_true", help="show the sketch's serial output")
    args = ap.parse_args()
    chips = emulated_chips(args.emulator)
    if not chips:
        print("FAIL esp32sim --list-chips reports no emulated chip")
        return 1
    if args.coverage:
        missing = [c for c in chips if not (PREBUILT / c / VARIANTS["uart"][0]).exists()]
        for c in chips:
            builds = [v for v, (image, _) in VARIANTS.items() if (PREBUILT / c / image).exists()]
            print(f"  {'ok  ' if c not in missing else 'FAIL'} {c}: {', '.join(builds) or 'no builds'}")
        if missing:
            print("Emulated chips without the Arduino suite: build it with "
                  f"python3 tools/build_arduino_compat.py --chip {' --chip '.join(missing)}")
        return 1 if missing else 0
    rc = 0
    for chip in [args.chip] if args.chip else chips:
        builds = [v for v, (image, _) in VARIANTS.items() if v == "uart" or (PREBUILT / chip / image).exists()]
        for variant in [args.variant] if args.variant else builds:
            print(f"== {chip} ({variant})")
            r = run_chip(args.emulator, chip, variant, args.verbose)
            rc = r if rc in (0, 77) and r else rc
    return rc


if __name__ == "__main__":
    sys.exit(main())
