# Arduino-ESP32 compatibility sketch

`arduino_compat/arduino_compat.ino` checks that Arduino-ESP32 sketches work on
the emulator. It is an ordinary Arduino sketch that tests itself: each test
runs in its own FreeRTOS task with a timeout and prints one result line, and
`tests/arduino_compat.py` runs it on every emulated chip (`ctest -R arduino`).
The same sketch can be flashed to a real board, apart from the tests that
expect the harness on the other end of the serial port.

`prebuilt/<chip>/` holds the images the tests use, built with Arduino-ESP32
3.3 (`manifest.json` records the core version and board options):

| File | Build | Emulator option |
|---|---|---|
| `arduino_compat.bin` | `Serial` on UART0 (DevKit default) | none |
| `arduino_compat_usbcdc.bin` | `Serial` on the USB-Serial-JTAG CDC port ("USB CDC On Boot", the XIAO default) | `--usb-console` |

Each image is merged (bootloader, partition table, `boot_app0` and app, from
offset 0) with the trailing erased flash trimmed, so the test writes it into
an empty flash file as esptool would.

## Tests

| Test | Checks |
|---|---|
| `chip_identity` | `ESP.getChipModel()` matches the build target, cores, CPU and crystal frequency, flash size, eFuse MAC, SDK version |
| `heap` | heap figures, a 64 KiB `malloc` shows in them, heap integrity |
| `cpp_runtime` | static constructors, exceptions, lambdas, STL containers |
| `string_and_math` | `String`, `printf` formatting, 64-bit and floating-point maths |
| `random` | `randomSeed` repeats, `random(min, max)` range, `esp_random`, `esp_fill_random` |
| `millis_micros` | `millis()`, `micros()` and `delay()` against each other |
| `cpu_frequency` | `setCpuFrequencyMhz()` 160/80/160, timing follows |
| `ticker` | `Ticker` periodic callbacks |
| `hw_timer` | `timerBegin`/`timerAlarm` interrupts at the right rate |
| `freertos` | tick rate, `vTaskDelay`, tasks, queues, mutexes, notifications |
| `std_thread` | `std::thread` and `std::atomic` |
| `gpio_output` | `digitalWrite` seen from outside on GPIO2 |
| `gpio_input_pulls` | pull-up and pull-down, and levels driven from outside on GPIO3 |
| `gpio_interrupt_external` | `attachInterrupt` on GPIO6, edges driven from outside |
| `gpio_interrupt_loopback` | interrupts from a pin's own output (GPIO7) |
| `serial_echo` | `Serial` reads a line typed by the host, and a 1000-byte burst |
| `serial1_loopback` | `Serial1` on GPIO0/1 with the UART's internal loopback |
| `preferences` | `Preferences` (NVS) write, read back, remove |
| `littlefs` | format, mount, write, read, list, rename, remove |
| `flash_raw` | `esp_partition` erase, write, read |
| `crypto` | SHA-256 through mbedTLS (SHA accelerator, typical and DMA), `MD5Builder` |
| `aes` | AES-256 ECB test vector, 4 KiB CBC and CTR round trips (AES accelerator over GDMA) |
| `ledc_pwm` | `ledcAttach`, `ledcWrite`, `ledcRead`, `ledcReadFreq`, `analogWrite`, `ledcDetach` on GPIO10 |
| `i2c_eeprom` | `Wire` on SDA 4 / SCL 5: an empty address NACKs, write and read back a 24C02 at 0x50 |
| `spi_transfer` | `SPI` on SCK 0 / MOSI 4 / MISO 5: idle MISO, then MOSI routed back to MISO for `transfer`, `transfer16/32`, `transferBytes` |
| `adc` | `analogRead` and `analogReadMilliVolts` on GPIO3, which reads full scale or 0 when driven high or low |
| `restart_persistence` | `ESP.restart()`, software reset reason, NVS and LittleFS data survive |

The pins are GPIO0-10, which every RISC-V ESP32 has.

## Protocol

The sketch prints lines starting with `@`; the harness reads them and plays
the outside world:

| Line | Meaning |
|---|---|
| `@BEGIN chip=… arduino=… idf=… serial=uart0\|usb-cdc` | sketch started |
| `@PASS name` / `@FAIL name detail` | one test's result |
| `@ASK DRIVE gpio 0\|1\|z` | drive a pin from outside (web UI `/api/pin`), answer `OK` |
| `@ASK PIN gpio 0\|1` | check a pin's level from outside (web UI events), answer `OK` or `NO` |
| `@ASK ECHO` / `@ASK BURST` | type a line, or a 1000-byte burst, on the serial port |
| `@NOTE text` | informational |
| `@RESTART` | the sketch restarts the chip; results continue after the boot |
| `@END` | all tests done |

`tests/arduino_compat.py` lists every test name and fails when one is not
reported, so a sketch that stops early cannot pass. The emulator runs with
`--i2c-dev eeprom@0x50` for the I2C test.

## Rebuilding

With arduino-cli and the Arduino-ESP32 core:

    arduino-cli config add board_manager.additional_urls \
        https://espressif.github.io/arduino-esp32/package_esp32_index.json
    arduino-cli core install esp32:esp32
    cmake --build build                      # the script asks esp32sim which chips are emulated
    python3 tools/build_arduino_compat.py    # or --chip esp32c6

## Adding a chip

Once `esp32sim --list-chips` reports a new chip as emulated, the
`arduino_coverage` test fails until it has a build here. Run
`python3 tools/build_arduino_compat.py --chip <chip>`, re-run CMake, and
ctest gains `arduino_<chip>_uart` (and `arduino_<chip>_usbcdc` when the board
has a USB CDC option). Add a test to the sketch and to `EXPECTED` in
`tests/arduino_compat.py` together, then rebuild every chip.
