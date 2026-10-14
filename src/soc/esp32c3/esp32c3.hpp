// ESP32-C3 system-on-chip: RV32IMC core, internal SRAM, flash behind the
// cache MMU, peripherals, and the (high-level emulated) mask ROM.
#pragma once
#include <deque>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/bus.hpp"
#include "core/rv32.hpp"
#include "soc/chip.hpp"
#include "soc/esp32c3/crypto.hpp"
#include "soc/esp32c3/gpio.hpp"
#include "soc/esp32c3/intmatrix.hpp"
#include "soc/esp32c3/timers.hpp"
#include "soc/esp32c3/peripherals.hpp"
#include "soc/spi_flash.hpp"
#include "util/elf.hpp"

namespace esp32sim {

class Esp32c3Rom;

namespace c3map {
constexpr uint32_t IRAM_BASE = 0x40370000, IRAM_SIZE = 0x70000;  // SRAM0 at +0xC000, SRAM1 at +0x10000
constexpr uint32_t SRAM0_BASE = 0x4037C000, SRAM1_IRAM = 0x40380000, SRAM_END_IRAM = 0x403E0000;
constexpr uint32_t DRAM_BASE = 0x3FC80000, DRAM_SIZE = 0x60000;  // alias of SRAM1
constexpr uint32_t RTC_FAST_BASE = 0x50000000, RTC_FAST_SIZE = 0x2000;
constexpr uint32_t IROM_BASE = 0x42000000, DROM_BASE = 0x3C000000, CACHE_WINDOW = 0x800000;
constexpr uint32_t ROM_BASE = 0x40000000, ROM_SIZE = 0x60000;
constexpr uint32_t DROM_MASK_BASE = 0x3FF00000, DROM_MASK_SIZE = 0x20000;
constexpr uint32_t PERIPH_BASE = 0x60000000, PERIPH_SIZE = 0x100000;
constexpr uint32_t UART0 = 0x60000000, SPI1 = 0x60002000, SPI0 = 0x60003000, GPIO = 0x60004000;
constexpr uint32_t RTC_CNTL = 0x60008000, EFUSE = 0x60008800, UART1 = 0x60010000;
constexpr uint32_t SYSCON = 0x60026000, SHA = 0x6003B000, TIMG0 = 0x6001F000, TIMG1 = 0x60020000;
constexpr uint32_t SYSTEM = 0x600C0000, INTMATRIX = 0x600C2000, IO_MUX = 0x60009000;
constexpr uint32_t SYSTIMER = 0x60023000, USB_SERIAL_JTAG = 0x60043000;
constexpr uint32_t MMU_TABLE = 0x600C5000, EXTMEM = 0x600C4000;
constexpr uint32_t ROM_STACK_TOP = 0x3FCDE710;
constexpr uint32_t ROM_VECTOR_TABLE = 0x40001D00;  // _vector_table in esp32c3_rev3_rom.elf
}  // namespace c3map

class Esp32c3 : public Chip, public Bus {
public:
    explicit Esp32c3(const ChipConfig& cfg);
    ~Esp32c3() override;

    // Chip
    std::string name() const override { return "ESP32-C3"; }
    void reset(BootMode mode) override;
    void hold_in_reset() override;
    void run_for(uint64_t cycles) override;
    void serial_input(const uint8_t* data, size_t len) override;
    uint32_t cpu_hz() const override { return cfg_.cpu_hz; }
    void shutdown() override;
    bool idle() const override { return state_ != State::Running; }
    std::string status() const override;
    uint64_t instructions() const override { return cpu_.instret; }

    // Bus slow path
    uint32_t mmio_read(uint32_t addr, int size) override;
    void mmio_write(uint32_t addr, uint32_t value, int size) override;

    // Used by the ROM emulation.
    Rv32Cpu& cpu() { return cpu_; }
    SpiFlash& flash() { return flash_; }
    c3::Mmu& mmu() { return *mmu_; }
    void console_write(const std::string& s);
    void console_write(const uint8_t* data, size_t len);
    void start_cpu(uint32_t entry);
    void halt(const std::string& why);
    void request_reset(BootMode mode, double delay_s = 0);
    uint64_t next_deadline(uint64_t target) const;
    uint32_t reset_reason() const;
    bool has_real_rom() const { return have_rom_; }
    std::string symbolize(uint32_t addr) const;
    void add_symbols(const std::string& elf_path) override;
    std::vector<int> user_gpios() const override;
    PinInfo pin_info(int gpio) const override { return gpio_->info(gpio); }
    void set_pin_input(int gpio, int level) override { gpio_->set_external(gpio, level); }
    uint64_t pins_version() const override { return gpio_->version(); }
    std::string where() const override;

private:
    enum class State { InReset, Loader, Running, Halted };

    void do_reset(BootMode mode, uint32_t reason);
    void attach(uint32_t base, Peripheral* p, uint32_t size = 0x1000, bool timed = false);
    struct PeriphSlot {
        Peripheral* p = nullptr;
        uint32_t base = 0;
        bool timed = false;  // writes may move a timer deadline
    };
    PeriphSlot& slot_for(uint32_t addr);
    PeriphSlot& create_generic_slot(uint32_t idx);
    void flush_output();
    void feed_uart_rx();
    void on_mmu_change(int index, uint32_t value);
    void on_fatal_trap(uint32_t cause, uint32_t tval);
    bool on_rom_call(Rv32Cpu& cpu);

    ChipConfig cfg_;
    Rv32Cpu cpu_;
    SpiFlash flash_;
    std::vector<uint8_t> sram_, rtc_fast_, rom_image_, drom_image_;
    ElfFile rom_elf_;
    bool have_rom_ = false;
    std::unordered_map<uint32_t, uint32_t> extra_csrs_;
    uint32_t mpccr_offset_ = 0;
    std::vector<ElfFile> symbol_files_;
    void load_rom_data();

    std::unique_ptr<c3::Uart> uart0_, uart1_;
    std::unique_ptr<c3::Gpio> gpio_;
    std::unique_ptr<c3::Systimer> systimer_;
    std::unique_ptr<c3::Efuse> efuse_;
    std::unique_ptr<c3::RtcCntl> rtc_cntl_;
    std::unique_ptr<c3::Mmu> mmu_;
    std::unique_ptr<c3::UsbSerialJtag> usb_jtag_;
    std::unique_ptr<c3::Extmem> extmem_;
    std::unique_ptr<c3::Syscon> syscon_;
    std::unique_ptr<c3::Sha> sha_;
    std::unique_ptr<c3::SystemReg> system_;
    std::unique_ptr<c3::IntMatrix> intmatrix_;
    std::unique_ptr<c3::Timg> timg0_, timg1_;
    c3::Clocks clocks_;
    std::vector<c3::TimedDevice*> timed_;
    void apply_clocks();
    void tick_timers();
    uint64_t next_timer_event() const;
    std::unique_ptr<SpiMemController> spi1_;
    std::vector<std::unique_ptr<Peripheral>> generic_;  // register files for not-yet-modelled blocks
    PeriphSlot efuse_slot_;
    PeriphSlot periph_map_[256];                        // 4 KiB granularity over 0x6000_0000..0x600F_FFFF
    std::unordered_set<uint32_t> warned_pages_;

    std::unique_ptr<Esp32c3Rom> rom_;
    State state_ = State::InReset;
    std::deque<uint8_t> host_rx_;
    std::vector<uint8_t> tx_buf_;
    bool reset_requested_ = false;
    uint64_t reset_at_cycles_ = 0;
    bool wdt_armed_ = false;
    uint64_t wdt_at_cycles_ = 0;
    BootMode requested_mode_ = BootMode::Flash;
    uint64_t flash_save_at_ = 0;
};

}  // namespace esp32sim
