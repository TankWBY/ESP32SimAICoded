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
