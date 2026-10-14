#!/usr/bin/env python3
"""Build the bare-metal CPU benchmark with clang + ld.lld + esptool.

Output: out/bench-ram.bin, a RAM-only image to place at flash offset 0x0
(the prebuilt copy lives in prebuilt/). Needs the same tools as
firmware/hello/build.py.
"""
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
HELLO = HERE.parent / "hello"
OUT = HERE / "out"
CFLAGS = ["--target=riscv32-unknown-elf", "-march=rv32imc", "-mabi=ilp32", "-mno-relax",
          "-O2", "-ffreestanding", "-fno-builtin", "-nostdlib", "-Wall", "-ffunction-sections"]


def run(*cmd):
    print(" ".join(str(c) for c in cmd))
    subprocess.run([str(c) for c in cmd], check=True)


def main():
    OUT.mkdir(exist_ok=True)
    objs = []
    for src in (HELLO / "start.S", HERE / "main.c", HERE / "kernels.c"):
        obj = OUT / (src.name + ".o")
        run("clang", *CFLAGS, "-c", src, "-o", obj)
        objs.append(obj)
    elf = OUT / "bench-ram.elf"
    run("ld.lld", "-T", HELLO / "ram.ld", "--gc-sections", "-o", elf, *objs)
    run(sys.executable, "-m", "esptool", "--chip", "esp32c3", "elf2image", "--flash-mode", "dio",
        "--flash-freq", "80m", "--flash-size", "4MB", "-o", OUT / "bench-ram.bin", elf)


if __name__ == "__main__":
    main()
