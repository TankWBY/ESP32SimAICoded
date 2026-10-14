// ESP32-C6 peripheral models that differ from the ESP32-C3 (register layouts
// from the ESP32-C6 TRM and ESP-IDF soc/esp32c6 headers). Blocks with the
// same layout on both chips (UART, GPIO, SYSTIMER, TIMG, SHA, eFuse, SPI1,
// USB-Serial-JTAG) reuse the c3:: models.
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>

#include "soc/esp32c3/intmatrix.hpp"
#include "soc/peripheral.hpp"

namespace esp32sim::c6 {

namespace irq {
constexpr int LP_WDT = 18, FROM_CPU0 = 22, GPIO = 30, UART0 = 43, UART1 = 44, USB_SERIAL_JTAG = 48;
constexpr int LEDC = 45, I2C_EXT0 = 50, TG0_T0 = 51, TG0_WDT = 53, TG1_T0 = 54, TG1_WDT = 56, SYSTIMER_TARGET0 = 57;
constexpr int APB_ADC = 60, DMA_IN_CH0 = 66, DMA_OUT_CH0 = 69, GSPI2 = 72, AES = 73, SHA = 74;
constexpr int kSources = 81;
}  // namespace irq

// Reset causes (LP_CLKRST_RESET_CAUSE, same codes as the C3's RTC_CNTL).
namespace reset {
constexpr uint32_t POWERON = 1, SW_SYS = 3, TG0_WDT_SYS = 7, TG1_WDT_SYS = 8, LP_WDT_SYS = 9, TG0_WDT_CPU = 11,
                   SW_CPU = 12, TG1_WDT_CPU = 17, LP_WDT_CPU = 13, SUPER_WDT = 18;
}

constexpr double kSlowClockHz = 136000.0;  // RC_SLOW, uncalibrated

// Power/clock/reset controller of the HP domain: clock tree (CPU, AHB, APB
// dividers) and peripheral clock/reset bits (plain storage).
class Pcr : public RegisterFile {
public:
    explicit Pcr(std::string name) : RegisterFile(std::move(name)) {}
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    uint32_t cpu_hz() const;
    // Clock of timer group `group`'s timer (or watchdog): XTAL, PLL_F80M or RC_FAST.
    uint32_t timg_hz(int group, bool wdt) const;
    std::function<void()> on_clock_change;
    std::function<void(int group)> before_timg_clock_change;
};

// LP clock/reset block: reports the reset cause.
class LpClkRst : public RegisterFile {
public:
    explicit LpClkRst(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    uint32_t reset_reason = reset::POWERON;
};

// LP always-on registers: STOREn (kept across software resets) and the
// software system / CPU reset triggers.
class LpAon : public RegisterFile {
public:
    explicit LpAon(std::string name) : RegisterFile(std::move(name)) {}
    void write(uint32_t off, uint32_t value) override;
    std::function<void(uint32_t reason)> on_sw_reset;
};

// LP (RTC) watchdog. Only stage 0 is modelled, as on the C3.
class LpWdt : public RegisterFile {
public:
    explicit LpWdt(std::string name) : RegisterFile(std::move(name)) {}
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    std::function<void(double delay_s)> on_wdt;  // < 0 disarms

private:
    void update();
};

// LP timer: 48-bit counter on the slow clock, latched by MAIN_TIMER_UPDATE.
class LpTimer : public RegisterFile {
public:
    explicit LpTimer(std::string name) : RegisterFile(std::move(name)) {}
    void write(uint32_t off, uint32_t value) override;
    std::function<uint64_t()> slow_ticks;
};

// L1 cache controller. The cache is not modelled (flash reads are always
// coherent), so maintenance operations complete at once.
class Cache : public RegisterFile {
public:
    explicit Cache(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
};

// SPI0 (cache-side flash controller). Holds the flash MMU: 256 entries
// reached through ITEM_INDEX/ITEM_CONTENT, page size 64/32/16 KiB.
class Spi0Mmu : public RegisterFile {
public:
    static constexpr int kEntries = 256;
    static constexpr uint32_t kValid = 1u << 9;
    explicit Spi0Mmu(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    void set_entry(int index, uint32_t value);
    uint32_t entry(int index) const { return table_[size_t(index)]; }
    uint32_t page_size() const;
    std::function<void(int index)> on_change;  // entry changed; -1: all (page size changed)

private:
    std::array<uint32_t, kEntries> table_{};
    uint32_t index_ = 0;
};

// Analog I2C master (BBPLL, bias, ... registers). Writes are remembered so
// read-modify-write sequences work; BBPLL calibration finishes at once.
class I2cAnaMst : public RegisterFile {
public:
    explicit I2cAnaMst(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;

private:
    std::unordered_map<uint32_t, uint8_t> analog_;
};

// PLIC_MX (0x2000_1000): the CPU interrupt line controller. Same registers as
// the C3's (enable, type, clear, priorities, threshold), 0x104 lower.
class Plic : public Peripheral {
public:
    Plic(std::string name, c3::IntMatrix& m) : Peripheral(std::move(name)), m_(m) {}
    uint32_t read(uint32_t off) override { return off <= 0x90 ? m_.cpu_read(off + 0x104) : regs_.read(off); }
    void write(uint32_t off, uint32_t value) override {
        if (off <= 0x90) m_.cpu_write(off + 0x104, value);
        else regs_.write(off, value);
    }
    void reset() override { regs_.reset(); }

private:
    c3::IntMatrix& m_;
    RegisterFile regs_{"PLIC"};
};

// INTPRI: the four FROM_CPU software interrupt sources (+0x90..+0x9C).
class Intpri : public RegisterFile {
public:
    explicit Intpri(std::string name) : RegisterFile(std::move(name)) {}
    void write(uint32_t off, uint32_t value) override;
    std::function<void(int index, bool level)> on_cpu_intr;
};

}  // namespace esp32sim::c6
