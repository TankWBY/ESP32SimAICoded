# CPU benchmark

A bare-metal ESP32-C3 program that measures how fast the emulator runs
CPU-bound code. It switches the CPU to 160 MHz (PLL), then runs rounds of 100
iterations of a CoreMark-like mix (`kernels.c`: linked-list walk, matrix
multiply, insertion sort, a text state machine, CRC-32 and recursive calls)
and prints one line per round:

```
bench: round 12 ok dcfd2453, 42323 us, 6771813 instructions
```

The checksum must equal the host build of the same kernels
(`cc -O2 host_check.c kernels.c && ./a.out`), so the line also checks the
CPU. The time comes from SYSTIMER and the count from `minstret`, both
emulated: they stay the same however fast the host is.

`tests/cpu_bench.py` (ctest `cpu_bench`) runs it with `--bench` and reports
the host speed as MIPS and as a percentage of a real 160 MHz chip:

```sh
python3 tests/cpu_bench.py --seconds 5
python3 tests/cpu_bench.py --min-speed 200   # fail below 2x real time
```

By hand: put `prebuilt/bench-ram.bin` at flash offset 0x0 (esptool
`write-flash 0x0`, or pad it into a flash file) and run
`esp32sim --bench 5 --echo`.

Rebuild with `python3 firmware/bench/build.py` (clang with the RISC-V target,
ld.lld and esptool, as for `firmware/hello`). It reuses `firmware/hello`'s
startup code, linker script and register header.
