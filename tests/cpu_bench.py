#!/usr/bin/env python3
"""CPU benchmark: run firmware/bench as fast as the host allows and report speed.

  python3 tests/cpu_bench.py [--emulator build/esp32sim] [--seconds 2] [--min-speed PCT]

Checks that every round's checksum matches the host build of the kernels and
that each round takes exactly the emulated time and instruction count it
always has (the CPU timing model must not drift). Prints the speed as MIPS
and as a percentage of a real 160 MHz ESP32-C3; --min-speed makes a speed
below PCT percent a failure (off by default: CI machines vary too much).
"""
import argparse
import os
import pathlib
import re
import socket
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent
IMAGE = ROOT / "firmware" / "bench" / "prebuilt" / "bench-ram.bin"
CHECKSUM = "dcfd2453"
ROUND_US, ROUND_INSNS = 42323, 6771813  # per round of 100 iterations at 160 MHz


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--emulator", default=str(ROOT / "build" / "esp32sim"))
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--min-speed", type=float, default=0)
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        flash = os.path.join(tmp, "flash.bin")
        image = IMAGE.read_bytes()
        with open(flash, "wb") as f:
            f.write(image + b"\xff" * (4 * 1024 * 1024 - len(image)))
        # Built-in ROM emulation: the RAM image boots the same with or without
        # Espressif's ROM, and the test needs no ROM download.
        r = subprocess.run([args.emulator, "--board", "esp32-c3-devkitm-1", "--flash", flash, "--no-rom",
                            "--tcp", str(free_port()), "--no-ui", "--no-console", "--echo",
                            "--bench", str(args.seconds)],
                           cwd=tmp, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=300)
    out = r.stdout
    rounds = re.findall(r"bench: round (\d+) (\w+) ([0-9a-f]{8}), (\d+) us, (\d+) instructions", out)
    speed = re.search(r"\((\d+)% of real time\), ([0-9.]+) MIPS", out)
    failures = []
    if r.returncode != 0:
        failures.append(f"emulator exited with {r.returncode}")
    expected_rounds = int(args.seconds * 1e6 / ROUND_US) - 1
    if len(rounds) < expected_rounds:
        failures.append(f"{len(rounds)} rounds finished, expected at least {expected_rounds}")
    for n, verdict, checksum, us, insns in rounds:
        if verdict != "ok" or checksum != CHECKSUM:
            failures.append(f"round {n}: checksum {checksum}, expected {CHECKSUM}")
        if int(us) != ROUND_US or int(insns) != ROUND_INSNS:
            failures.append(f"round {n}: {us} us / {insns} instructions, expected {ROUND_US} us / {ROUND_INSNS}")
    if not speed:
        failures.append("no speed report")
    else:
        pct, mips = int(speed.group(1)), float(speed.group(2))
        print(f"speed: {mips:.1f} MIPS, {pct}% of a real 160 MHz ESP32-C3")
        if args.min_speed and pct < args.min_speed:
            failures.append(f"speed {pct}% is below --min-speed {args.min_speed:g}%")
    for f in failures[:10]:
        print("FAIL " + f)
    if failures:
        print(out[-2000:])
        print(r.stderr[-2000:])
        return 1
    print(f"ok: {len(rounds)} rounds, checksums and timing as expected")
    return 0


if __name__ == "__main__":
    sys.exit(main())
