// High-level emulation of the ESP32-C3 mask ROM.
//
// The real chip boots from a 384 KiB mask ROM that contains the first-stage
// bootloader, the UART download protocol used by esptool, and a library of
// routines (printf, SPI flash driver, cache setup, ...) that firmware calls.
// Espressif does not ship that ROM in a redistributable form, so - like many
// console emulators do with a BIOS - we reimplement its observable behaviour:
//
//  * reset banner on UART0 ("ESP-ROM:esp32c3-...", rst/boot reason)
//  * download mode: the SLIP serial protocol spoken by esptool
//  * flash boot: parse the ESP image at 0x0, load RAM segments, configure the
//    cache MMU for flash-mapped segments and jump to the entry point
//
// ROM routines called by firmware are dispatched to call(); they are not
// implemented yet, so such firmware halts with a clear message.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "soc/chip.hpp"

namespace esp32sim {

class Esp32c3;
class Rv32Cpu;

class Esp32c3Rom {
public:
    explicit Esp32c3Rom(Esp32c3& soc) : soc_(soc) {}

    void on_reset(BootMode mode);
    void feed(const uint8_t* data, size_t len);  // UART0 RX while in download mode
    bool call(Rv32Cpu& cpu);                     // HLE of ROM functions

private:
    struct Segment {
        uint32_t addr, len, flash_offset;
    };
    struct Image {
        uint32_t entry = 0;
        uint8_t spi_mode = 0, spi_speed_size = 0;
        uint16_t chip_id = 0;
        std::vector<Segment> segments;
    };

    // Download mode
    void handle_packet(const std::vector<uint8_t>& p);
    void respond(uint8_t op, uint32_t val, const std::vector<uint8_t>& data, uint8_t status = 0, uint8_t error = 0);
    void finish_deflate(uint8_t* status, uint8_t* error);

    // Flash boot
    void boot_flash();
    bool parse_image(uint32_t offset, Image& img, std::string* error);
    bool find_app_partition(uint32_t* offset, std::string* label);
    void load_and_run(const Image& img, bool print_rom_log);

    Esp32c3& soc_;
    bool download_mode_ = false;

    // SLIP decoder state
    bool in_frame_ = false, escape_ = false;
    std::vector<uint8_t> frame_;

    // MEM_* state
    uint32_t mem_offset_ = 0, mem_block_size_ = 0, mem_size_ = 0;
    // FLASH_* state
    uint32_t flash_offset_ = 0, flash_block_size_ = 0, flash_blocks_ = 0, flash_erase_size_ = 0;
    // FLASH_DEFL_* state
    bool defl_active_ = false;
    uint32_t defl_blocks_expected_ = 0, defl_blocks_got_ = 0;
    std::vector<uint8_t> defl_buf_;
};

}  // namespace esp32sim
