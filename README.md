# esp32sim

A standalone ESP32 hardware emulator, written in C++, that other software sees
as an ESP32 on a serial port. Tools such as esptool, serial monitors and IDEs
connect to it the same way they connect to a dev board.

It works the way console emulators like SNES9X do. It runs the firmware's real
machine code on an emulated CPU, with emulated memory, buses and peripherals.
It does not fake responses at the application level.

**Status: proof of concept.** Two chips are emulated: the **ESP32-C3** and the
**ESP32-C6**. Both boot unmodified ESP-IDF firmware (the real 2nd stage
bootloader, FreeRTOS, `esp_restart()`) when given Espressif's ROM image, which
is bundled in `roms/`.

A built-in web UI (http://localhost:8080/ while it runs) shows the board with
live pin states and lets you drive inputs, press BOOT and RESET, switch boards
and use the serial console.

## What works today

| Area | State |
|---|---|
| Chips | ESP32-C3 and ESP32-C6 (no radios, no C6 LP core) |
| CPU | RV32IMAC + Zicsr machine mode, traps, vectored interrupts, `wfi`, pre-decoded instruction cache, Espressif cycle counter CSRs |
| Mask ROM | Espressif's real ROM image (`roms/`, Apache-2.0) runs natively, so firmware can call any ROM routine; boot and download mode are high-level emulated on top |
| Clocks | CPU frequency follows the clock registers (C3 SYSTEM, C6 PCR: XTAL, PLL 80/160 MHz); time is counted in emulated cycles |
| Interrupts | Interrupt matrix (C3 62 sources, C6 81) and the CPU interrupt controller (C6: PLIC): enable, level/edge, priority, threshold |
| Memory map | C3: SRAM (IRAM/DRAM aliases), RTC fast RAM. C6: 512 KiB HP SRAM, 16 KiB LP SRAM. Both: mask-ROM window, peripheral space |
| Flash | SPI NOR model (JEDEC ID, read, page program, sector/block/chip erase, status regs), stored in a file |
| Cache MMU | C3: 128 × 64 KiB pages, IROM `0x4200_0000` and DROM `0x3C00_0000`. C6: 256 pages of 64/32/16 KiB at `0x4200_0000`. Code and data run in place from flash |
| SPI1 controller | User commands plus the fixed hardware commands, which is how esptool reads the flash ID |
| UART0/1 | RX/TX FIFOs, status, RX interrupts; UART0 is bridged to the host serial endpoints |
| SYSTIMER | 16 MHz units 0/1, three comparators (one-shot and periodic) with interrupts: the FreeRTOS tick and esp_timer |
| Timer groups | TIMG0/1 timer with alarm/auto-reload, main watchdogs (interrupt and reset stages), RTC clock calibration |
| SHA | SHA-1/224/256 accelerator (used by the bootloader to verify app images) |
| RNG | Hardware random number register |
| GPIO | GPIO matrix and IO_MUX: outputs, inputs, pulls, open drain, peripheral routing, edge/level interrupts, strap pins; pads can be driven from the UI or console |
| eFuse | MAC address and chip revision (C3 v0.4, C6 v0.1) |
| RTC / LP domain | Software system and CPU reset, RTC watchdog with feed/disable, RTC timer, reset reason (C3 RTC_CNTL; C6 LP_AON, LP_WDT, LP_TIMER, LP_CLKRST) |
| USB-Serial-JTAG | Console output is forwarded to the same serial endpoints |
| ROM boot | Reset banner, flash boot of an ESP image (RAM segments + MMU mapping), ROM "direct boot", partition table lookup |
| ROM download mode | The esptool serial protocol: sync, register read/write, RAM download, flash write (plain and **compressed**), MD5, flash read, security info |
| Reset lines | DTR/RTS over RFC 2217 drive an emulated dev-board auto-reset circuit, so esptool resets into download mode by itself |
| Boards | 242 boards from PlatformIO's `platform-espressif32`; picks chip, flash size and CPU clock |

Tested end to end with **esptool v5.5.0**: chip detection, flash ID, compressed
multi-file `write-flash` with verification (with esptool's flasher stub when the
ROM image is present, or `--no-stub` without it), watchdog reset, and automatic
reset over RFC 2217. ESP-IDF v5.3.1's `hello_world` then boots through its own
bootloader, prints its output and restarts itself in a loop, on both chips
(`tests/idf_hello.py --chip esp32c3|esp32c6`). A bare-metal demo also runs
without any ROM image (ESP32-C3).

## Not there yet (roadmap)

1. **More peripherals.** I2C, SPI2, LEDC, RMT, ADC, GDMA and
   the USB-Serial-JTAG receive side are register stubs, so firmware that
   relies on them will not behave yet. Wi-Fi and Bluetooth are out of scope
   for now: the radio would need a much larger model.
2. **Arduino.** Arduino-ESP32 is built on ESP-IDF, so it should get as far as
   IDF does, but it has not been tried yet.
3. **More chips.** The ESP32-H2 is close to the C6 and should come next. The
   ESP32, S2 and S3 need an Xtensa LX6/LX7 core.
4. **Speed.** The emulator runs at about 110-150 MIPS here, roughly 70-95% of
   a real 160 MHz C3. Timers follow emulated cycles, so firmware timing stays
   correct even when wall-clock speed is lower, and an idle FreeRTOS app
   (waiting in `wfi`) costs almost no host CPU.
5. **Windows testing.** The Windows build cross-compiles with MinGW-w64, and
   `--com` uses the standard Win32 serial API, but neither has been run on
   Windows yet, and MSVC has not been tried.

## Building

You need CMake 3.16 or newer and a C++17 compiler. There are no third-party
dependencies.

```sh
# Linux / macOS
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build            # unit tests (+ esptool and ESP-IDF end-to-end tests if esptool is installed)
```

```bat
:: Windows (Visual Studio 2019+ or MinGW-w64)
cmake -S . -B build
cmake --build build --config Release
```

The Linux build and tests run with GCC 13 and Clang 18. A Windows executable
cross-compiles cleanly with MinGW-w64 GCC 13, but it has not been run on
Windows or built with MSVC yet.

## Running

```sh
./build/esp32sim --board esp32-c3-devkitm-1 --rfc2217 5555 --tcp 5556 --pty /tmp/ttyESP32
./build/esp32sim --board esp32-c6-devkitc-1        # ESP32-C6 (also esp32-c6-devkitm-1, seeed_xiao_esp32c6)
```

* `--rfc2217 PORT` gives you `rfc2217://localhost:PORT`. DTR and RTS are
  carried, so esptool's normal auto-reset works. (pyserial's RFC 2217 client
  makes many round trips, so esptool is slow over it, about 20 s for a small
  flash.)
* `--tcp PORT` gives you `socket://localhost:PORT`. It is fast, but carries no
  reset lines.
* `--pty PATH` (Linux/macOS) creates a real device path for tools that only
  accept serial ports. It carries no reset lines.
* `--com PORT` opens a serial device: one end of a virtual COM port pair, so
  tools see a normal `COMx` port on Windows (see below) or a tty on Linux.
  DTR/RTS are carried, so esptool's automatic reset works.
* `--rom FILE` picks the mask-ROM image. By default the chip's image in `roms/`
  (`esp32c3_rev3_rom.elf`, `esp32c6_rev0_rom.elf`) is used when present. `--no-rom` forces the built-in ROM emulation only, which
  is enough for bare-metal firmware. `--elf FILE` adds symbols (for example
  your app's ELF) to crash dumps and `-v` speed logs.
* The flash contents persist in `flash-<board>.bin`, or the file you pass with
  `--flash`.
* While it runs, type `r` + Enter to reset and run, `d` + Enter to reset into
  download mode, `pins` to list GPIO states, `p 4 0` / `p 4 1` / `p 4 z` to
  drive GPIO4 low, high or release it, `q` to quit.
* `--echo` mirrors the chip's serial output to the terminal. `-v` shows debug
  logs, including emulation speed.

### Web UI

The web UI is on by default at http://localhost:8080/ (`--ui PORT` to move it,
`--no-ui` to turn it off, `--open` to open it in your browser). It is a single
page compiled into the executable, so there is nothing else to install.

* **Family and board pickers.** The family list comes from the chip catalogue
  (ESP32-C3 and ESP32-C6 are emulated; the others are listed but disabled). The
  board list is every PlatformIO board of that family. Boards with a picture
  come first: the Espressif ESP32-C3-DevKitM-1, ESP32-C3-DevKitC-02,
  ESP32-C6-DevKitC-1 and ESP32-C6-DevKitM-1 (Espressif's own board drawings),
  and the Seeed Studio XIAO ESP32C3 and XIAO ESP32C6 (drawings made for this
  project, with a link to Seeed's wiki; for the C3 the page also shows Seeed's
  official photo and pinout). The XIAO ESP32C6's yellow user LED (GPIO15)
  lights up on the picture. Other boards get a generic view of the chip's
  GPIOs. Switching boards restarts the chip with that board's flash file
  (`flash-<board>.bin`).
* **Pins.** Each pin shows its level (high, low or floating) and how the
  firmware configured it: output, input, pull-up/down, open drain, the
  peripheral using it (such as U0TXD), and its interrupt type. Click a pin on
  the board, or use the table, to drive it high or low from outside, or
  release it. If you and the firmware drive a pin to different levels, the
  chip's output wins and the pin is flagged.
* **BOOT and RESET.** Both are press-and-hold, on the board picture or in the
  header. Hold BOOT (GPIO9 low), press and release RESET, and the chip starts in
  download mode, as on real hardware.
* **Serial console.** Shows UART0 output with ESP-IDF log colours and sends
  what you type. It works alongside the RFC 2217/TCP/COM endpoints.

`firmware/gpio-demo` is a small ESP-IDF app made for trying it: blinking
outputs, a button input, an edge interrupt and the BOOT button.

Board layouts live in `boards/layouts/<board>.json` (pin positions on the
picture in `boards/images/`). Adding a board is one JSON file plus an image.

### Flashing with esptool

With the ROM image present, esptool works with its defaults, flasher stub
included. Without it (`--no-rom`), add `--no-stub`.

```sh
# automatic reset, like a USB dev board
esptool --port rfc2217://localhost:5555 write-flash 0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 app.bin

# fast path: start the emulator with --boot download (or type "d"), then
esptool --port socket://localhost:5556 --before no-reset --after watchdog-reset \
        write-flash 0x0 firmware/idf-hello/prebuilt/bootloader.bin \
        0x8000 firmware/idf-hello/prebuilt/partition-table.bin 0x10000 firmware/idf-hello/prebuilt/hello_world.bin
```

`idf.py -p rfc2217://localhost:5555 flash monitor` and PlatformIO
(`upload_port = rfc2217://localhost:5555`) work the same way.

### Serial monitors

Anything that can open `socket://`, `rfc2217://` or a device path works. For
example: `python -m serial.tools.miniterm rfc2217://localhost:5555 115200`,
`idf.py monitor -p rfc2217://localhost:5555`, PlatformIO with
`monitor_port = rfc2217://localhost:5555`, or `screen /tmp/ttyESP32 115200`.

### Virtual COM port

Some tools only list real serial ports. `--com` makes the emulator show up as
one, using a virtual null-modem pair: two linked ports where whatever is
written to one comes out of the other.

**Windows** (com0com, free and signed):

1. Install [com0com](https://com0com.sourceforge.net/). In its setup tool,
   create a pair and rename the ports, for example `COM20` and `COM21`. Tick
   "use Ports class" so that every tool lists them.
2. Start the emulator on one end: `esp32sim --com COM20`
3. Point your tool at the other end, `COM21`: Arduino IDE, PlatformIO,
   `esptool --port COM21 ...`, `idf.py -p COM21 flash monitor`, PuTTY.

com0com connects the tool's DTR/RTS to the emulator's DSR/CTS, so esptool's
reset into download mode works like a USB dev board.

**Linux**: tools that take a device path can use `--pty /tmp/ttyESP32`. For
DTR/RTS as well, load the [tty0tty](https://github.com/freemed/tty0tty)
module and run `esp32sim --com /dev/tnt0`, then use `/dev/tnt1`.

`--com` was tested on Linux with a pty pair (esptool chip detection). The
Windows code path compiles but has not been run on Windows yet.

## Demo firmware

`firmware/idf-hello/prebuilt` (ESP32-C3) and `prebuilt-esp32c6` are ESP-IDF
v5.3.1's unmodified `hello_world` example (bootloader, partition table, app).
They need the ROM image. See `firmware/idf-hello/README.md`.

`firmware/gpio-demo` is an ESP-IDF app for the web UI's pin view (source, and
prebuilt images for both chips). See `firmware/gpio-demo/README.md`.

`firmware/hello` is a bare-metal C program that uses only hardware registers,
no ROM calls, so it runs on real silicon and in the emulator alike. It prints
the eFuse MAC, blinks GPIO8 using SYSTIMER, echoes input, and restarts the chip
when you type `reset`. Prebuilt images are in `firmware/hello/prebuilt/`:

| File | Flash at | Notes |
|---|---|---|
| `hello-ram.bin` | `0x0` | loaded into SRAM by the ROM, like a 2nd-stage bootloader |
| `partitions.bin` | `0x8000` | `nvs`, `phy_init`, `factory` @ `0x10000` |
| `hello-xip.bin` | `0x10000` | code and constants run in place from flash via the MMU |

With all three flashed and `--no-rom`, boot goes through the bootloader +
partition table + app path, with the 2nd-stage bootloader step emulated. To
rebuild: `python3 firmware/hello/build.py`
(needs clang with the RISC-V target, ld.lld and esptool).

## Boards

```sh
./build/esp32sim --list-boards c3     # filter by text
./build/esp32sim --list-chips         # emulation status per chip family
./build/esp32sim --pio-boards         # read boards straight from your PlatformIO install
python3 tools/update_boards.py        # refresh boards/boards.json (local PIO, git, or GitHub API)
```

`boards/boards.json` is a compact cache of PlatformIO's board manifests:
MCU, CPU clock, flash size, RAM, name and vendor.

## Layout

```
src/core/        RV32 CPU interpreter, memory bus (16 KiB page table + MMIO slow path)
src/soc/         chip interface, shared RISC-V SoC core (riscv_soc), ROM high-level emulation (esp_rom),
                 SPI flash + SPI_MEM controller, auto-reset circuit
src/soc/esp32c3/ ESP32-C3 SoC; its peripherals, timers, GPIO, interrupt matrix and SHA are shared with the C6
src/soc/esp32c6/ ESP32-C6 SoC: PCR clocks, LP domain, PLIC, flash MMU, cache, pad table
src/host/        serial endpoints: RFC 2217 / raw TCP server, POSIX PTY, serial devices (COM ports); web UI server
ui/              the web UI page (embedded into the executable at build time)
boards/layouts/  board pictures' pin positions for the UI; boards/images/ the pictures
src/boards/      PlatformIO board catalogue
src/util/        zlib inflate, MD5, JSON and ELF readers, logging
tests/           unit tests (C++), end-to-end esptool, ESP-IDF and web UI tests (Python)
firmware/        ESP-IDF hello_world and GPIO demo (prebuilt), bare-metal demo firmware
roms/            Espressif mask-ROM images (Apache-2.0)
tools/           board list updater, ROM downloader
```

New chip variants implement `esp32sim::Chip` (`src/soc/chip.hpp`) and register
in `create_chip()`. RISC-V chips derive from `RiscvSoc` and only add their
memory map and peripherals, as `Esp32c6` does.
