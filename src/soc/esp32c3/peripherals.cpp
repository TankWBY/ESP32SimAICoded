#include "soc/esp32c3/peripherals.hpp"
#include <algorithm>

#include "util/log.hpp"

namespace esp32sim::c3 {

// ---------------------------------------------------------------- UART
namespace uart {
constexpr uint32_t FIFO = 0x00, INT_RAW = 0x04, INT_ST = 0x08, INT_ENA = 0x0C, INT_CLR = 0x10;
constexpr uint32_t CLKDIV = 0x14, STATUS = 0x1C, CONF0 = 0x20, CONF1 = 0x24, DATE = 0x7C, ID = 0x80;
constexpr uint32_t RXFIFO_FULL = 1u << 0, TXFIFO_EMPTY = 1u << 1, RXFIFO_TOUT = 1u << 8;
}  // namespace uart

void Uart::reset() {
    RegisterFile::reset();
    rx_.clear();
    int_latched_ = 0;
    update_irq();
    set_default(uart::CLKDIV, 0x2B6);  // 80 MHz / 115200
    set_default(uart::CONF0, 0x1000001C);
    set_default(uart::CONF1, 0x0000C060);
    set_default(uart::DATE, 0x2008270);
    set_default(uart::ID, 0x500);
}

uint32_t Uart::int_raw() {
    uint32_t thr = RegisterFile::read(uart::CONF1) & 0x1FF;
    uint32_t raw = int_latched_ | uart::TXFIFO_EMPTY;
    if (!rx_.empty() && rx_.size() >= (thr ? thr : 1)) raw |= uart::RXFIFO_FULL;
    if (!rx_.empty()) raw |= uart::RXFIFO_TOUT;
    return raw;
}

void Uart::update_irq() {
    if (irq) irq((int_raw() & RegisterFile::read(uart::INT_ENA)) != 0);
}

uint32_t Uart::read(uint32_t off) {
    switch (off) {
    case uart::FIFO: {
        if (rx_.empty()) return 0;
        uint8_t b = rx_.front();
        rx_.pop_front();
        if (rx_.empty()) update_irq();
        return b;
    }
    case uart::INT_RAW: return int_raw();
    case uart::INT_ST: return int_raw() & RegisterFile::read(uart::INT_ENA);
    case uart::STATUS: return uint32_t(rx_.size()) & 0x3FF;  // TX FIFO drains instantly
    default: return RegisterFile::read(off);
    }
}

void Uart::write(uint32_t off, uint32_t value) {
    switch (off) {
    case uart::FIFO:
        if (on_tx) on_tx(uint8_t(value));
        return;
    case uart::INT_CLR:
        int_latched_ &= ~value;
        update_irq();
        return;
    case uart::CONF0:
        if (value & (1u << 17)) rx_.clear();  // RXFIFO_RST
        break;
    }
    RegisterFile::write(off, value);
    if (off == uart::INT_ENA || off == uart::CONF0 || off == uart::CONF1) update_irq();
}

// ---------------------------------------------------------------- EFUSE
Efuse::Efuse(std::string name, const std::array<uint8_t, 6>& mac) : RegisterFile(std::move(name)) {
    // BLOCK1 (EFUSE_RD_MAC_SPI_SYS_n) starts at +0x44.
    set_default(0x44, (uint32_t(mac[2]) << 24) | (uint32_t(mac[3]) << 16) | (uint32_t(mac[4]) << 8) | mac[5]);
    set_default(0x48, (uint32_t(mac[0]) << 8) | mac[1]);
    // Word 3: wafer version minor low bits [20:18] = 4 -> chip revision v0.4,
    // pkg_version [23:21] = 0 (QFN32), flash_cap [29:27] = 0 (external flash).
    set_default(0x50, 4u << 18);
    set_default(0x58, 0);  // word 5: minor hi bit, major revision 0
    set_default(0x1FC, 0x2007200);  // DATE
}

// ---------------------------------------------------------------- RTC_CNTL
uint32_t RtcCntl::read(uint32_t off) {
    if (off == 0x38) return reset_reason | (reset_reason << 6);  // RESET_STATE: procpu/appcpu cause
    if (off == 0x74) return RegisterFile::read(off) & ~(1u << 31);  // SLOW_CLK_NEXT_EDGE completes at once
    return RegisterFile::read(off);
}

void RtcCntl::write(uint32_t off, uint32_t value) {
    if (off == 0x00 && (value & (1u << 31))) {  // OPTIONS0.SW_SYS_RST
        LOGI("rtc: software system reset requested");
        reset_reason = 3;  // RTC_SW_SYS_RESET
        if (on_system_reset) on_system_reset(0);
        return;
    }
    if (off == 0x00 && (value & (1u << 5))) {  // OPTIONS0.SW_PROCPU_RST
        LOGI("rtc: software CPU reset requested");
        reset_reason = 12;  // RTC_SW_CPU_RESET
        if (on_system_reset) on_system_reset(0);
        return;
    }
    if (off == 0x0C && (value & (1u << 31))) {  // TIME_UPDATE: latch the RTC timer
        uint64_t t = slow_ticks ? slow_ticks() : 0;
        RegisterFile::write(0x10, uint32_t(t));
        RegisterFile::write(0x14, uint32_t(t >> 32) & 0xFFFF);
        return;
    }
    if (off >= 0x90 && off <= 0xA4) {  // RWDT registers, guarded by WDTWPROTECT
        if (RegisterFile::read(0xA8) != 0x50D83AA1) return;
        if (off == 0xA4) {  // WDTFEED
            if (value & (1u << 31)) update_wdt();
            return;
        }
        RegisterFile::write(off, value);
        if (off == 0x90 || off == 0x94) update_wdt();
        return;
    }
    RegisterFile::write(off, value);
}

// Only stage 0 is modelled. Its action field (WDTCONFIG0[30:28]) is 0 = off,
// 1 = interrupt, 2..5 = some flavour of reset; all resets are reported as
// RTCWDT_RTC_RESET. Feeding or reconfiguring restarts the count.
void RtcCntl::update_wdt() {
    uint32_t cfg0 = RegisterFile::read(0x90);
    uint32_t action = (cfg0 >> 28) & 7;
    if (!(cfg0 & (1u << 31)) || action < 2) {
        if (on_wdt) on_wdt(-1);
        return;
    }
    double timeout = std::max(RegisterFile::read(0x94), 1u) / kSlowClockHz;
    LOGD("rtc: RTC watchdog (re)started, resets in %.0f ms unless fed", timeout * 1000);
    if (on_wdt) on_wdt(timeout);
}

// ---------------------------------------------------------------- SYSTEM
namespace sysreg {
constexpr uint32_t CPU_PER_CONF = 0x08, CPU_INTR_FROM_CPU0 = 0x28, SYSCLK_CONF = 0x58;
}

void SystemReg::reset() {
    RegisterFile::reset();
    set_default(sysreg::CPU_PER_CONF, 0x0C);          // PLL 480 MHz, CPU period 80 MHz
    set_default(sysreg::SYSCLK_CONF, 1 | (40u << 12)); // XTAL/2, CLK_XTAL_FREQ = 40
    set_default(0x00, 0);
    set_default(0x10, 0xF9C1E06F);  // PERIP_CLK_EN0
    set_default(0x18, 0);           // PERIP_RST_EN0
    if (on_clock_change) on_clock_change();
}

uint32_t SystemReg::cpu_hz() const {
    uint32_t sys = read_const(sysreg::SYSCLK_CONF);
    uint32_t div = (sys & 0x3FF) + 1;
    switch ((sys >> 10) & 3) {
    case 0: return 40000000u / div;
    case 1: return (read_const(sysreg::CPU_PER_CONF) & 3) ? 160000000u : 80000000u;
    default: return 17500000u / div;
    }
}

uint32_t SystemReg::apb_hz() const {
    return ((read_const(sysreg::SYSCLK_CONF) >> 10) & 3) == 1 ? 80000000u : cpu_hz();
}

void SystemReg::write(uint32_t off, uint32_t value) {
    if (off == sysreg::SYSCLK_CONF) value = (value & ~(0x7Fu << 12)) | (40u << 12);  // XTAL freq is read-only
    RegisterFile::write(off, value);
    if (off == sysreg::CPU_PER_CONF || off == sysreg::SYSCLK_CONF) {
        if (on_clock_change) on_clock_change();
    } else if (off >= sysreg::CPU_INTR_FROM_CPU0 && off < sysreg::CPU_INTR_FROM_CPU0 + 16) {
        if (on_cpu_intr) on_cpu_intr(int(off - sysreg::CPU_INTR_FROM_CPU0) / 4, value & 1);
    }
}

// ---------------------------------------------------------------- SYSCON
uint32_t Syscon::read(uint32_t off) {
    if (off == 0xB0) {  // WDEV_RND: xorshift64*, never returns the same value twice
        rng_state_ ^= rng_state_ >> 12;
        rng_state_ ^= rng_state_ << 25;
        rng_state_ ^= rng_state_ >> 27;
        return uint32_t((rng_state_ * 0x2545F4914F6CDD1Dull) >> 32);
    }
    return RegisterFile::read(off);
}

// ---------------------------------------------------------------- MMU
void Mmu::reset() {
    for (int i = 0; i < kEntries; ++i) set_entry(i, kInvalid);
}

uint32_t Mmu::read(uint32_t off) {
    uint32_t i = off / 4;
    return i < kEntries ? table_[i] : 0;
}

void Mmu::write(uint32_t off, uint32_t value) {
    uint32_t i = off / 4;
    if (i < kEntries) set_entry(int(i), value & 0x1FF);
}

void Mmu::set_entry(int index, uint32_t value) {
    table_[size_t(index)] = value;
    if (on_change) on_change(index, value);
}

// ---------------------------------------------------------------- EXTMEM
uint32_t Extmem::read(uint32_t off) {
    uint32_t v = RegisterFile::read(off);
    switch (off) {
    case 0x1C: return v | (1u << 2);   // ICACHE_LOCK_CTRL: lock done
    case 0x28: return v | (1u << 1);   // ICACHE_SYNC_CTRL: sync/invalidate done
    case 0x34: return v | (1u << 1);   // ICACHE_PRELOAD_CTRL: preload done
    case 0x40: return v | (1u << 3);   // ICACHE_AUTOLOAD_CTRL: autoload done
    case 0xB0: return 1;               // CACHE_STATE: icache FSM idle
    case 0xCC: return (v & ~4u) | ((v & 1) << 2);  // ICACHE_FREEZE: done follows enable
    default: return v;
    }
}

// ---------------------------------------------------------------- USB_SERIAL_JTAG
uint32_t UsbSerialJtag::read(uint32_t off) {
    if (off == 0x04) return 0x2;  // EP1_CONF: SERIAL_IN_EP_DATA_FREE
    return RegisterFile::read(off);
}

void UsbSerialJtag::write(uint32_t off, uint32_t value) {
    if (off == 0x00) {  // EP1: byte to host
        if (on_tx) on_tx(uint8_t(value));
        return;
    }
    RegisterFile::write(off, value);
}

}  // namespace esp32sim::c3
