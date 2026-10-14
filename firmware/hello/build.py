#!/usr/bin/env python3
"""Build the bare-metal ESP32-C3 demo with clang + ld.lld + esptool.

Outputs (in ./out):
  hello-ram.bin        RAM-only image, flash at 0x0 (booted directly by the ROM)
  hello-xip.bin        execute-in-place app, flash at 0x10000
  partitions.bin       partition table for the XIP layout, flash at 0x8000

Needs: clang with the RISC-V target, ld.lld, and `pip install esptool`.
"""
import hashlib
import pathlib
import shutil
import struct
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "out"
CFLAGS = ["--target=riscv32-unknown-elf", "-march=rv32imc", "-mabi=ilp32", "-mno-relax",
          "-Os", "-ffreestanding", "-fno-builtin", "-nostdlib", "-Wall", "-ffunction-sections"]


def run(*cmd):
    print(" ".join(str(c) for c in cmd))
    subprocess.run([str(c) for c in cmd], check=True)


def tool(name, *alternatives):
    for n in (name,) + alternatives:
        if shutil.which(n):
            return n
    sys.exit(f"{name} not found in PATH")


def partition_table():
    entries = [  # (label, type, subtype, offset, size)
        ("nvs", 1, 0x02, 0x9000, 0x6000),
        ("phy_init", 1, 0x01, 0xF000, 0x1000),
        ("factory", 0, 0x00, 0x10000, 0x100000),
    ]
    table = b"".join(struct.pack("<2sBBII16sI", b"\xaa\x50", t, st, off, size, label.encode(), 0)
                     for label, t, st, off, size in entries)
    table += b"\xeb\xeb" + b"\xff" * 14 + hashlib.md5(table).digest()
    return table + b"\xff" * (0xC00 - len(table))


def main():
    clang = tool("clang")
    lld = tool("ld.lld")
    OUT.mkdir(exist_ok=True)
    objs = []
    for src in ("start.S", "main.c"):
        obj = OUT / (src + ".o")
        run(clang, *CFLAGS, "-c", HERE / src, "-o", obj)
        objs.append(obj)
    for variant in ("ram", "xip"):
        elf = OUT / f"hello-{variant}.elf"
        run(lld, "-T", HERE / f"{variant}.ld", "--gc-sections", "-o", elf, *objs)
        run(sys.executable, "-m", "esptool", "--chip", "esp32c3", "elf2image", "--flash-mode", "dio",
            "--flash-freq", "80m", "--flash-size", "4MB", "-o", OUT / f"hello-{variant}.bin", elf)
    (OUT / "partitions.bin").write_bytes(partition_table())
    print("\nFlash with, for example:")
    print("  esptool --port rfc2217://localhost:5555 --no-stub write-flash 0x0 out/hello-ram.bin")


if __name__ == "__main__":
    main()
