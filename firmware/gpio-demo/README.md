# GPIO demo (for the web UI)

A small ESP-IDF v5.3.1 app for ESP32-C3 that exercises the pins you see in the
board view. Pin numbers are chip GPIOs; on a Seeed XIAO ESP32C3 they are the
pads D0, D1, D2, D3 and D10.

| GPIO | XIAO | Mode | What it does |
|---|---|---|---|
| 2  | D0  | output | blinks once per second |
| 10 | D10 | output | blinks twice per second |
| 3  | D1  | output | follows GPIO4, inverted (drive GPIO4 low and it lights) |
| 4  | D2  | input, pull-up | a "button": drive it low in the UI, the console prints `button GPIO4 pressed` |
| 5  | D3  | input, pull-down, interrupt on any edge | prints `GPIO5 changed, now 1/0` |
| 9  | D9  | input, pull-up, falling-edge interrupt | the BOOT button: prints `BOOT button pressed` |

Flash it (the emulator starts in download mode, then boots the app):

    esp32sim --tcp 5556 --boot download
    esptool --port socket://localhost:5556 --before no-reset --after watchdog-reset \
        write-flash 0x0 prebuilt/bootloader.bin 0x8000 prebuilt/partition-table.bin 0x10000 prebuilt/gpio_demo.bin

Then open http://localhost:8080/.

To rebuild: `idf.py set-target esp32c3 build` here, then copy
`build/bootloader/bootloader.bin`, `build/partition_table/partition-table.bin`
and `build/gpio_demo.bin` into `prebuilt/`.
