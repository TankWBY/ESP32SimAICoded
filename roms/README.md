# Mask ROM images

`esp32c3_rev3_rom.elf` is Espressif's ROM ELF for ESP32-C3 revision v0.3 and
later, from the esp-rom-elfs release 20241011
(https://github.com/espressif/esp-rom-elfs). Espressif publishes these files
under the Apache License 2.0 (see `LICENSE` here).

The emulator loads it automatically when it sits next to the executable in
`roms/` (or pass `--rom FILE`). ESP-IDF and Arduino firmware call into the ROM
constantly, so they need it; the bundled bare-metal demos do not.

`python3 tools/fetch_roms.py` downloads the archive again (checksum-verified)
and extracts the ROMs for every chip.
