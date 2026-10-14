#!/usr/bin/env python3
"""End-to-end check with real ESP-IDF firmware: flash the prebuilt hello_world
(bootloader, partition table, app) with esptool, let it boot on the real ROM
and FreeRTOS, and wait for its output and its esp_restart() reboot.

  python3 tests/idf_hello.py [--emulator build/esp32sim] [--chip esp32c3|esp32c6]

The ESP32-C3 run uses esptool's flasher stub and its watchdog reset; the
ESP32-C6 run uses the ROM loader (--no-stub) and, since esptool has no
watchdog reset for the C6 and a TCP port has no RTS line, the emulator's
console reset command. That keeps both download paths covered.

Exits 77 (skipped) when the ROM image or esptool is missing.
"""
import argparse
import os
import pathlib
import socket
import subprocess
import sys
import tempfile
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
CHIPS = {
    "esp32c3": dict(fw="prebuilt", rom="esp32c3_rev3_rom.elf", board="esp32-c3-devkitm-1", stub=True, tcp=5571,
                    revision=b"silicon revision v0.4"),
    "esp32c6": dict(fw="prebuilt-esp32c6", rom="esp32c6_rev0_rom.elf", board="esp32-c6-devkitc-1", stub=False,
                    tcp=5573, revision=b"silicon revision v0.1"),
}


def main():
    ap = argparse.ArgumentParser()
    exe = "esp32sim.exe" if os.name == "nt" else "esp32sim"
    ap.add_argument("--emulator", default=str(ROOT / "build" / exe))
    ap.add_argument("--chip", default="esp32c3", choices=sorted(CHIPS))
    args = ap.parse_args()
    c = CHIPS[args.chip]
    FW = ROOT / "firmware" / "idf-hello" / c["fw"]
    ROM = ROOT / "roms" / c["rom"]
    TCP = c["tcp"]
    if not ROM.exists():
        print(f"skip: {ROM} missing (python3 tools/fetch_roms.py)")
        return 77
    try:
        import esptool  # noqa: F401
    except ImportError:
        print("skip: esptool not installed")
        return 77

    with tempfile.TemporaryDirectory() as tmp:
        flash = os.path.join(tmp, "flash.bin")
        emu = subprocess.Popen([args.emulator, "--flash", flash, "--tcp", str(TCP), "--boot", "download",
                                *(["--no-console"] if c["stub"] else []), "--no-gpio-log", "--no-ui", "--rom", str(ROM), "--board", c["board"]],
                               stdin=subprocess.DEVNULL if c["stub"] else subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        try:
            time.sleep(0.5)
            port = f"socket://localhost:{TCP}"
            r = subprocess.run([sys.executable, "-m", "esptool", "--port", port, "--before", "no-reset",
                                "--after", "watchdog-reset" if c["stub"] else "no-reset", *([] if c["stub"] else ["--no-stub"]),
                                "write-flash",
                                "0x0", str(FW / "bootloader.bin"), "0x8000", str(FW / "partition-table.bin"),
                                "0x10000", str(FW / "hello_world.bin")], capture_output=True, text=True, timeout=120)
            if r.stdout.count("Hash of data verified") != 3:
                print(r.stdout + r.stderr)
                print("FAIL flashing")
                return 1
            print("  ok   bootloader, partition table and app flashed")
            s = socket.create_connection(("localhost", TCP))
            if not c["stub"]:
                emu.stdin.write("r\n")  # console command: reset and run
                emu.stdin.flush()
            s.settimeout(0.2)
            want = [b"boot: ESP-IDF v5.3.1 2nd stage bootloader", b"main_task: Calling app_main()",
                    b"Hello world!", c["revision"], b"Restarting now.", b"rst:0xc (RTC_SW_CPU_RST)"]
            out = b""
            deadline = time.time() + 40  # the app counts down 10 s before restarting
            while time.time() < deadline and not all(w in out for w in want):
                try:
                    out += s.recv(65536)
                except socket.timeout:
                    pass
            failed = 0
            for w in want:
                ok = w in out
                failed += not ok
                print(f"  {'ok  ' if ok else 'FAIL'} {w.decode()}")
            if failed:
                print(out.decode(errors="replace")[-3000:])
            return 1 if failed else 0
        finally:
            if emu.stdin:
                try:
                    emu.stdin.write("q\n")
                    emu.stdin.close()
                except OSError:
                    pass
            emu.terminate()
            emu.wait(timeout=5)


if __name__ == "__main__":
    sys.exit(main())
