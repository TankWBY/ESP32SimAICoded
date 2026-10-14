// ESP32-C6 system-on-chip: RV32IMAC core, 512 KiB HP SRAM, 16 KiB LP SRAM,
// flash behind the cache MMU, peripherals, and the mask ROM. The radios
// (Wi-Fi 6, BLE, 802.15.4) and the LP core are not emulated.
#pragma once
#include <memory>
#include <vector>

#include "soc/esp32c3/crypto.hpp"
#include "soc/esp32c3/gpio.hpp"
#include "soc/esp32c3/intmatrix.hpp"
#include "soc/esp32c3/peripherals.hpp"
#include "soc/esp32c3/timers.hpp"
#include "soc/esp32c6/peripherals.hpp"
#include "soc/riscv_soc.hpp"

namespace esp32sim {

namespace c6map {
constexpr uint32_t HP_SRAM_BASE = 0x40800000, HP_SRAM_SIZE = 0x80000;
constexpr uint32_t LP_SRAM_BASE = 0x50000000, LP_SRAM_SIZE = 0x4000;
constexpr uint32_t CACHE_BASE = 0x42000000, CACHE_WINDOW = 0x1000000;
constexpr uint32_t ROM_BASE = 0x40000000, ROM_SIZE = 0x50000;
constexpr uint32_t PERIPH_BASE = 0x60000000, PERIPH_SIZE = 0xD0000;
constexpr uint32_t UART0 = 0x60000000, UART1 = 0x60001000, SPI0 = 0x60002000, SPI1 = 0x60003000;
constexpr uint32_t TIMG0 = 0x60008000, TIMG1 = 0x60009000, SYSTIMER = 0x6000A000;
constexpr uint32_t USB_SERIAL_JTAG = 0x6000F000, INTMTX = 0x60010000, SHA = 0x60089000;
constexpr uint32_t IO_MUX = 0x60090000, GPIO = 0x60091000, PCR = 0x60096000, I2C_ANA_MST = 0x600AF800;
constexpr uint32_t LP_CLKRST = 0x600B0400, EFUSE = 0x600B0800, LP_TIMER = 0x600B0C00, LP_AON = 0x600B1000;
constexpr uint32_t LP_WDT = 0x600B1C00, LPPERI = 0x600B2800, INTPRI = 0x600C5000, EXTMEM = 0x600C8000;
constexpr uint32_t PLIC_MX = 0x20001000, CLINT_M = 0x20001800;
constexpr uint32_t ROM_STACK_TOP = 0x4087E610;     // end of .stack_pro in esp32c6_rev0_rom.elf
constexpr uint32_t ROM_VECTOR_TABLE = 0x40001500;  // _vector_table
}  // namespace c6map

class Esp32c6 : public RiscvSoc {
public:
    explicit Esp32c6(const ChipConfig& cfg);
    ~Esp32c6() override;

    std::string name() const override { return "ESP32-C6"; }
    std::vector<int> user_gpios() const override;
    uint32_t reset_reason() const override { return lp_clkrst_->reset_reason; }
    void set_mmu_entry(int index, uint32_t page) override { spi0_->set_entry(index, page | c6::Spi0Mmu::kValid); }

protected:
    void reset_peripherals(BootMode mode, uint32_t reason) override;
    void set_reset_reason(uint32_t reason) override { lp_clkrst_->reset_reason = reason; }
    c3::Uart& console_uart() override { return *uart0_; }
    uint32_t rom_stack_top() const override { return c6map::ROM_STACK_TOP; }
    void on_cpu_start() override;
    void init_rom_state() override;

private:
    void remap_mmu(int index);
    void map_mmu_entry(int index, uint32_t page_size);
    void apply_clocks();

    std::vector<uint8_t> hp_sram_, lp_sram_, rom_image_;
    uint32_t mapped_page_size_ = 0x10000;
    std::unique_ptr<c3::Uart> uart0_, uart1_;
    std::unique_ptr<c3::Gpio> gpio_dev_;
    std::unique_ptr<c3::Systimer> systimer_;
    std::unique_ptr<c3::Timg> timg0_, timg1_;
    std::unique_ptr<c3::IntMatrix> intmtx_;
    std::unique_ptr<c3::Efuse> efuse_;
    std::unique_ptr<c3::UsbSerialJtag> usb_jtag_;
    std::unique_ptr<c3::Syscon> lpperi_;
    std::unique_ptr<c3::Sha> sha_;
    std::unique_ptr<SpiMemController> spi1_;
    std::unique_ptr<c6::Spi0Mmu> spi0_;
    std::unique_ptr<c6::Pcr> pcr_;
    std::unique_ptr<c6::LpClkRst> lp_clkrst_;
    std::unique_ptr<c6::LpAon> lp_aon_;
    std::unique_ptr<c6::LpWdt> lp_wdt_;
    std::unique_ptr<c6::LpTimer> lp_timer_;
    std::unique_ptr<c6::Cache> cache_;
    std::unique_ptr<c6::I2cAnaMst> i2c_ana_;
    std::unique_ptr<c6::Plic> plic_;
    std::unique_ptr<c6::Intpri> intpri_;
    std::unique_ptr<RegisterFile> clint_;
};

}  // namespace esp32sim
