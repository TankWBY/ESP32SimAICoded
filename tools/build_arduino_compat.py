#!/usr/bin/env python3
"""Builds the Arduino compatibility sketch (firmware/arduino-compat) for the
chips the test suite runs it on, with arduino-cli and the Arduino-ESP32 core,
and stores the images in firmware/arduino-compat/prebuilt/<chip>/.

  python3 tools/build_arduino_compat.py                 # every emulated chip
  python3 tools/build_arduino_compat.py --chip esp32h2  # one chip

Needs arduino-cli with the esp32:esp32 core installed:

  arduino-cli config add board_manager.additional_urls \\
      https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core install esp32:esp32

Each chip gets a "uart" build (Serial on UART0) and, when the board menu has a
"USB CDC On Boot" option, a "usbcdc" build (Serial on the USB-Serial-JTAG
port). Images are merged (bootloader, partition table, boot_app0 and app at
offset 0, trailing erased flash trimmed), so the test writes one file into the
emulator's flash. manifest.json records what was built with which core.
"""
import argparse
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent
SKETCH = ROOT / "firmware" / "arduino-compat" / "arduino_compat"
PREBUILT = ROOT / "firmware" / "arduino-compat" / "prebuilt"
# Build name -> (image file, extra FQBN options).
VARIANTS = {"uart": ("arduino_compat.bin", ""), "usbcdc": ("arduino_compat_usbcdc.bin", "CDCOnBoot=cdc")}


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def status(line):
    """The STATUS column of a `--list-chips` row (the first of its words)."""
    m = re.search(r"\s(emulated|not yet)\b", line)
    return m and m.group(1)


def emulated_chips():
    exe = ROOT / "build" / ("esp32sim.exe" if os.name == "nt" else "esp32sim")
    if not exe.exists():
        sys.exit(f"{exe} not found: build the emulator first, or pass --chip")
    out = run([str(exe), "--list-chips"]).stdout
    return [line.split()[0] for line in out.splitlines()[1:] if status(line) == "emulated"]


def core_version(cli):
    out = run([cli, "core", "list", "--json"]).stdout
    try:
        data = json.loads(out)
        platforms = data.get("platforms", data) if isinstance(data, dict) else data
        for p in platforms:
            if p.get("id") == "esp32:esp32":
                return p.get("installed_version") or p.get("installed") or "?"
    except (ValueError, AttributeError):
        pass
    return "?"


def has_cdc_option(cli, fqbn):
    out = run([cli, "board", "details", "-b", fqbn]).stdout
    return "CDCOnBoot" in out


def build(cli, chip, variant):
    image, opts = VARIANTS[variant]
    fqbn = f"esp32:esp32:{chip}" + (f":{opts}" if opts else "")
    with tempfile.TemporaryDirectory() as out:
        r = run([cli, "compile", "--fqbn", fqbn, "--output-dir", out, str(SKETCH)])
        merged = pathlib.Path(out) / "arduino_compat.ino.merged.bin"
        if r.returncode or not merged.exists():
            print(r.stdout + r.stderr)
            sys.exit(f"build failed: {fqbn}")
        data = merged.read_bytes().rstrip(b"\xff")
    dest = PREBUILT / chip / image
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(data)
    print(f"  {fqbn}: {dest.relative_to(ROOT).as_posix()} ({len(data)} bytes)")
    return fqbn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chip", action="append", help="chip to build for (repeatable; default: every emulated chip)")
    ap.add_argument("--arduino-cli", default=shutil.which("arduino-cli") or "arduino-cli")
    args = ap.parse_args()
    cli = args.arduino_cli
    if not shutil.which(cli) and not pathlib.Path(cli).exists():
        sys.exit("arduino-cli not found (see the instructions at the top of this script)")
    version = core_version(cli)
    for chip in args.chip or emulated_chips():
        print(f"== {chip}")
        builds = {"uart": build(cli, chip, "uart")}
        if has_cdc_option(cli, f"esp32:esp32:{chip}"):
            builds["usbcdc"] = build(cli, chip, "usbcdc")
        manifest = {"arduino_esp32": version, "builds": {v: {"image": VARIANTS[v][0], "fqbn": f}
                                                         for v, f in builds.items()}}
        (PREBUILT / chip / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
