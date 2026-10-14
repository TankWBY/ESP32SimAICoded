# ESP-IDF hello_world (real firmware)

`prebuilt/` holds ESP-IDF v5.3.1's unmodified `examples/get-started/hello_world`
for ESP32-C3 with the default sdkconfig: the real 2nd stage bootloader, the
partition table and the app. It runs FreeRTOS, prints chip info, counts down
and restarts through `esp_restart()` forever.

It needs the ROM image (`roms/esp32c3_rev3_rom.elf`). Flash and watch it:

    esp32sim --tcp 5556 --boot download
    esptool --port socket://localhost:5556 --before no-reset --after watchdog-reset --no-stub \
        write-flash 0x0 prebuilt/bootloader.bin 0x8000 prebuilt/partition-table.bin 0x10000 prebuilt/hello_world.bin

To rebuild: `idf.py set-target esp32c3 build` in the example directory, then
copy `build/bootloader/bootloader.bin`, `build/partition_table/partition-table.bin`
and `build/hello_world.bin` here.

`prebuilt-esp32c6/` is the same example built for ESP32-C6 (`idf.py set-target
esp32c6 build`, default sdkconfig: 2 MB flash, so 32 KiB MMU pages). It needs
`roms/esp32c6_rev0_rom.elf`. esptool has no watchdog reset for the C6 and a TCP
port has no reset line, so flash with `--after no-reset` and then type `r` in
the emulator console (or use `--rfc2217` and esptool's default reset):

    esp32sim --board esp32-c6-devkitc-1 --tcp 5556 --boot download
    esptool --port socket://localhost:5556 --before no-reset --after no-reset \
        write-flash 0x0 prebuilt-esp32c6/bootloader.bin 0x8000 prebuilt-esp32c6/partition-table.bin \
        0x10000 prebuilt-esp32c6/hello_world.bin
