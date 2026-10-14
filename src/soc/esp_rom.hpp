// High-level emulation of the mask-ROM boot path shared by the RISC-V ESP32
// chips (ESP32-C3, ESP32-C6).
//
// The real chip boots from a mask ROM that contains the first-stage
// bootloader, the UART download protocol used by esptool, and a library of
// routines (printf, SPI flash driver, cache setup, ...) that firmware calls.
// When Espressif's ROM image is loaded (roms/), firmware calls those routines
// natively; the boot and download paths are reimplemented here either way:
//
//  * reset banner on UART0 ("ESP-ROM:esp32c3-...", rst/boot reason)
//  * download mode: the SLIP serial protocol spoken by esptool
//  * flash boot: parse the ESP image at 0x0, load RAM segments, configure the
//    cache MMU for flash-mapped segments and jump to the entry point
//
// Without a ROM image, calls into the ROM area land in call(), which halts
// the chip with a clear message.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "core/bus.hpp"
#include "soc/chip.hpp"

namespace esp32sim {

class Rv32Cpu;
class SpiFlash;

// What a chip tells the ROM emulation about itself.
struct RomProfile {
    std::string chip_name;          // "ESP32-C3"
    std::string banner;             // first lines printed after reset
    uint16_t image_chip_id = 0;     // chip id in ESP image headers and GET_SECURITY_INFO
    uint32_t boot_flash = 0xC;      // "boot:0x.." value printed for a flash boot
    uint32_t boot_download = 0x4;   // ... and for download mode
    struct Range {
        uint32_t base, size;
    };
    std::vector<Range> ram;         // where image segments may be loaded
    std::vector<Range> cache;       // flash windows behind the cache MMU
    uint32_t mmu_entries = 128;     // 64 KiB pages per window
    std::string rom_file;           // e.g. "roms/esp32c3_rev3_rom.elf", for messages
};

// The SoC services the ROM emulation needs.
class RomHost : public Bus {
public:
    virtual void console_write(const std::string& s) = 0;
    virtual SpiFlash& flash() = 0;
    virtual void halt(const std::string& why) = 0;
    virtual bool has_real_rom() const = 0;
    virtual void request_reset(BootMode mode, double delay_s = 0) = 0;
    virtual uint32_t reset_reason() const = 0;
    virtual void start_cpu(uint32_t entry) = 0;
    // Point MMU entry `index` (64 KiB page of every cache window) at flash page `page`.
    virtual void set_mmu_entry(int index, uint32_t page) = 0;
};

class EspRom {
public:
    EspRom(RomHost& soc, RomProfile profile) : soc_(soc), p_(std::move(profile)) {}

    void on_reset(BootMode mode);
    void feed(const uint8_t* data, size_t len);  // UART0 RX while in download mode
    bool call(Rv32Cpu& cpu);                     // ROM routines (none emulated)
    const RomProfile& profile() const { return p_; }

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
    const RomProfile::Range* cache_window(uint32_t addr) const;
    bool in_ram(uint32_t addr) const;

    RomHost& soc_;
    RomProfile p_;
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
