#!/usr/bin/env python3
"""End-to-end check: real esptool + serial monitor against a running esp32sim.

  python3 tests/esptool_flow.py [--emulator build/esp32sim] [--rfc2217]

Steps: start the emulator in download mode, read chip info with esptool,
flash the prebuilt demo (bootloader slot, partition table, XIP app), reboot via
esptool's watchdog reset, then talk to the running firmware over TCP.
With --rfc2217 it also checks esptool's automatic DTR/RTS reset (slower).
Needs `pip install esptool`.
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
FW = ROOT / "firmware" / "hello" / "prebuilt"
TCP, RFC = 15556, 15555


def esptool(port, *args, before="no-reset", after="no-reset"):
    cmd = [sys.executable, "-m", "esptool", "--port", port, "--before", before, "--after", after, "--no-stub", *args]
    print("$ esptool", " ".join(args))
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit("esptool failed")
    return r.stdout


def expect(cond, what):
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        raise SystemExit(1)


def read_until(sock, needle, timeout=10.0):
    data, end = b"", time.time() + timeout
    while needle not in data and time.time() < end:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk
        except socket.timeout:
            pass
    return data


def main():
    ap = argparse.ArgumentParser()
    exe = "esp32sim.exe" if os.name == "nt" else "esp32sim"
    ap.add_argument("--emulator", default=str(ROOT / "build" / exe))
    ap.add_argument("--rfc2217", action="store_true", help="also test auto-reset over RFC 2217")
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        flash = os.path.join(tmp, "flash.bin")
        emu = subprocess.Popen([args.emulator, "--flash", flash, "--tcp", str(TCP), "--rfc2217", str(RFC),
                                "--boot", "download", "--no-console", "--no-gpio-log", "--no-rom", "--no-ui"],
                               stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        try:
            time.sleep(0.5)
            port = f"socket://localhost:{TCP}"
            out = esptool(port, "chip-id")
            expect("ESP32-C3" in out, "esptool detects an ESP32-C3")
            expect("7c:df:a1:5e:00:01" in out, "MAC address read from eFuse")

            out = esptool(port, "flash-id")
            expect("Detected flash size: 4MB" in out, "flash ID / size detection over SPI registers")

            out = esptool(port, "write-flash", "--compress", "0x0", str(FW / "hello-ram.bin"),
                          "0x8000", str(FW / "partitions.bin"), "0x10000", str(FW / "hello-xip.bin"),
                          after="watchdog-reset")
            expect(out.count("Hash of data verified") == 3, "three images written (compressed) and MD5-verified")

            s = socket.create_connection(("localhost", TCP))
            s.settimeout(0.2)
            # Watchdog reset already booted the app; reset again via the
            # firmware itself to capture a full boot log on this connection.
            s.sendall(b"reset\r")
            boot = read_until(s, b"Type something")
            expect(b"rst:0x3 (RTC_SW_SYS_RST)" in boot, "software reset reported by ROM banner")
            expect(b"booting partition 'factory' at 0x10000" in boot, "app partition located via partition table")
            expect(b"Hello from a bare-metal ESP32-C3 program!" in boot, "XIP firmware prints its banner")
            expect(b"Running from 0x420000" in boot, "code executes from flash through the MMU (IROM)")
            s.sendall(b"ping\r")
            expect(b"You typed: ping" in read_until(s, b"You typed: ping"), "UART RX/TX echo round trip")
            s.close()

            if args.rfc2217:
                out = esptool(f"rfc2217://localhost:{RFC}", "chip-id", before="default-reset", after="hard-reset")
                expect("ESP32-C3" in out, "RFC 2217: DTR/RTS reset into download mode and back")
        finally:
            emu.terminate()
            try:
                _, err = emu.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                emu.kill()
                _, err = emu.communicate()
            if "halted" in err:
                print(err)
    print("all end-to-end checks passed")


if __name__ == "__main__":
    main()
