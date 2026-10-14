// Common base of the RISC-V ESP32 chips (ESP32-C3, ESP32-C6): the CPU, SPI
// flash, mask ROM image, peripheral address decoding, the run loop with its
// timer scheduling, reset handling and diagnostics. Each chip adds its memory
// map and peripherals.
#pragma once
#include <deque>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/rv32.hpp"
#include "soc/chip.hpp"
#include "soc/esp32c3/gpio.hpp"
#include "soc/esp32c3/peripherals.hpp"
#include "soc/esp32c3/timers.hpp"
#include "soc/esp_rom.hpp"
#include "soc/spi_flash.hpp"
#include "util/elf.hpp"

namespace esp32sim {

class RiscvSoc : public Chip, public RomHost {
public:
    RiscvSoc(const ChipConfig& cfg, uint32_t periph_base, uint32_t periph_size);
    ~RiscvSoc() override;

    // Chip
    void reset(BootMode mode) override;
    void hold_in_reset() override;
    void run_for(uint64_t cycles) override;
    void serial_input(const uint8_t* data, size_t len) override;
    uint32_t cpu_hz() const override { return cfg_.cpu_hz; }
    void shutdown() override;
    bool idle() const override { return state_ != State::Running; }
    std::string status() const override;
    uint64_t instructions() const override { return cpu_.instret; }
    std::string where() const override;
    void add_symbols(const std::string& elf_path) override;
    PinInfo pin_info(int gpio) const override { return gpio_->info(gpio); }
    void set_pin_input(int gpio, int level) override { gpio_->set_external(gpio, level); }
    uint64_t pins_version() const override { return gpio_->version(); }

    // Bus slow path
    uint32_t mmio_read(uint32_t addr, int size) override;
    void mmio_write(uint32_t addr, uint32_t value, int size) override;

    // RomHost
    void console_write(const std::string& s) override;
    SpiFlash& flash() override { return flash_; }
    void halt(const std::string& why) override;
    bool has_real_rom() const override { return have_rom_; }
    void request_reset(BootMode mode, double delay_s = 0) override;
    void start_cpu(uint32_t entry) override;

    Rv32Cpu& cpu() { return cpu_; }
    std::string symbolize(uint32_t addr) const;

protected:
    enum class State { InReset, Loader, Running, Halted };

    // Chip-specific parts.
    virtual void reset_peripherals(BootMode mode, uint32_t reason) = 0;
    virtual void set_reset_reason(uint32_t reason) = 0;
    virtual c3::Uart& console_uart() = 0;
    virtual uint32_t rom_stack_top() const = 0;
    virtual void on_cpu_start() {}       // state the real ROM leaves behind
    virtual void init_rom_state() {}     // after the ROM's .data/.bss are loaded
    virtual bool is_power_on_reset(uint32_t reason) const { return reason == 1 || reason == 16; }

    // Setup helpers for the chip constructors.
    void attach(uint32_t base, Peripheral* p, uint32_t size = 0x1000, bool timed = false);
    void attach_region(uint32_t base, uint32_t size, Peripheral* p);  // outside the main peripheral window
    // Load the ROM ELF's code/rodata into `image` (mapped at `base`), and
    // remember which sections are .data/.bss for each reset.
    void load_rom(uint32_t base, std::vector<uint8_t>& image);
    void finish_setup(RomProfile profile, uint32_t rom_base, uint32_t rom_size);
    void arm_rtc_wdt(double delay_s);  // < 0 disarms
    void apply_cpu_clock(uint32_t hz);
    uint32_t rom_symbol(const std::string& name) const;
    void write_rom_word(const std::string& symbol, uint32_t value, int size = 4);

    ChipConfig cfg_;
    Rv32Cpu cpu_;
    SpiFlash flash_;
    ElfFile rom_elf_;
    bool have_rom_ = false;
    c3::Clocks clocks_;
    std::vector<c3::TimedDevice*> timed_;
    c3::Gpio* gpio_ = nullptr;
    int boot_pin_ = 9;  // strapping pin sampled at reset (the BOOT button)
    State state_ = State::InReset;
    std::unique_ptr<EspRom> rom_;
    std::vector<uint8_t> tx_buf_;
    bool reset_requested_ = false;

private:
    struct PeriphSlot {
        Peripheral* p = nullptr;
        uint32_t base = 0;
        bool timed = false;  // writes may move a timer deadline
    };
    struct Region {
        uint32_t base, size;
        Peripheral* p;
    };
    void do_reset(BootMode mode, uint32_t reason);
    PeriphSlot& slot_for(uint32_t addr);
    PeriphSlot& create_generic_slot(uint32_t idx);
    Region* region_for(uint32_t addr);
    void load_rom_data();
    void flush_output();
    void feed_uart_rx();
    void tick_timers();
    uint64_t next_timer_event() const;
    uint64_t next_deadline(uint64_t target) const;
    void on_fatal_trap(uint32_t cause, uint32_t tval);
    bool on_rom_call(Rv32Cpu& cpu);

    uint32_t periph_base_, periph_size_;
    uint32_t rom_base_ = 0, rom_size_ = 0;
    std::vector<PeriphSlot> periph_map_;  // 1 KiB granularity
    std::vector<Region> regions_;
    std::vector<std::unique_ptr<Peripheral>> generic_;  // register files for not-yet-modelled blocks
    std::unordered_set<uint32_t> warned_pages_;
    std::unordered_map<uint32_t, uint32_t> extra_csrs_;
    uint32_t mpccr_offset_ = 0;
    std::vector<ElfFile> symbol_files_;
    std::deque<uint8_t> host_rx_;
    uint64_t reset_at_cycles_ = 0;
    bool wdt_armed_ = false;
    uint64_t wdt_at_cycles_ = 0;
    BootMode requested_mode_ = BootMode::Flash;
    uint64_t flash_save_at_ = 0;
};

}  // namespace esp32sim
