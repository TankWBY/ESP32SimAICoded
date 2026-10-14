// ESP32-C3 system-on-chip: RV32IMC core, internal SRAM, flash behind the
// cache MMU, peripherals, and the mask ROM.
#pragma once
#include <memory>
#include <vector>

#include "soc/esp32c3/crypto.hpp"
#include "soc/esp32c3/gpio.hpp"
#include "soc/esp32c3/intmatrix.hpp"
#include "soc/esp32c3/peripherals.hpp"
#include "soc/esp32c3/timers.hpp"
#include "soc/riscv_soc.hpp"

namespace esp32sim {

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

class Esp32c3 : public RiscvSoc {
public:
    explicit Esp32c3(const ChipConfig& cfg);
    ~Esp32c3() override;

    std::string name() const override { return "ESP32-C3"; }
    std::vector<int> user_gpios() const override;
    uint32_t reset_reason() const override { return rtc_cntl_->reset_reason; }
    void set_mmu_entry(int index, uint32_t page) override { mmu_->set_entry(index, page); }

protected:
    void reset_peripherals(BootMode mode, uint32_t reason) override;
    void set_reset_reason(uint32_t reason) override { rtc_cntl_->reset_reason = reason; }
    c3::Uart& console_uart() override { return *uart0_; }
    uint32_t rom_stack_top() const override { return c3map::ROM_STACK_TOP; }
    void on_cpu_start() override;
    void init_rom_state() override;

private:
    void on_mmu_change(int index, uint32_t value);
    void apply_clocks();

    std::vector<uint8_t> sram_, rtc_fast_, rom_image_, drom_image_;
    std::unique_ptr<c3::Uart> uart0_, uart1_;
    std::unique_ptr<c3::Gpio> gpio_dev_;
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
    std::unique_ptr<SpiMemController> spi1_;
};

}  // namespace esp32sim
