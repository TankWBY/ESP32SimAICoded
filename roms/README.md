# Mask ROM images

`esp32c3_rev3_rom.elf` (ESP32-C3 revision v0.3 and later) and
`esp32c6_rev0_rom.elf` (every ESP32-C6 revision) are Espressif's ROM ELFs from
the esp-rom-elfs release 20241011
(https://github.com/espressif/esp-rom-elfs). Espressif publishes these files
under the Apache License 2.0 (see `LICENSE` here).

The emulator loads the right one automatically when it sits next to the executable in
`roms/` (or pass `--rom FILE`). ESP-IDF and Arduino firmware call into the ROM
constantly, so they need it; the bundled bare-metal demos do not.

`python3 tools/fetch_roms.py` downloads the archive again (checksum-verified)
and extracts the ROMs for every chip.
