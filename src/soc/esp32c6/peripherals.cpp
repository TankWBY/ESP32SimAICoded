#include "soc/esp32c6/peripherals.hpp"

#include <algorithm>

#include "util/log.hpp"

namespace esp32sim::c6 {

// ---------------------------------------------------------------- PCR
namespace pcr {
constexpr uint32_t SYSCLK_CONF = 0x110, CPU_FREQ_CONF = 0x118, AHB_FREQ_CONF = 0x11C, APB_FREQ_CONF = 0x120,
                   SYSCLK_FREQ_QUERY_0 = 0x124, DATE = 0xFFC;
constexpr uint32_t TG0_TIMER_CLK = 0x40, TG0_WDT_CLK = 0x44, TG1_TIMER_CLK = 0x4C, TG1_WDT_CLK = 0x50;
}

void Pcr::reset() {
    RegisterFile::reset();
    set_default(pcr::SYSCLK_CONF, (2u << 8) | (40u << 24));  // XTAL, HS_DIV_NUM = 2, CLK_XTAL_FREQ = 40
    set_default(pcr::SYSCLK_FREQ_QUERY_0, (480u << 8) | 20u);  // PLL 480 MHz, RC_FAST 20 MHz
    set_default(pcr::DATE, 0x2208150);
    if (on_clock_change) on_clock_change();
}

uint32_t Pcr::cpu_hz() const {
    const uint32_t sys = read_const(pcr::SYSCLK_CONF), cpu = read_const(pcr::CPU_FREQ_CONF);
    switch ((sys >> 16) & 3) {
    case 0: return 40000000u / (((sys & 0xFF) + 1) * ((cpu & 0xFF) + 1));
    case 1: {
        uint32_t hs = (cpu >> 8) & 0xFF;
        if (hs == 0 && (cpu & (1u << 16))) return 120000000u;  // CPU_HS_120M_FORCE
        return 480000000u / ((((sys >> 8) & 0xFF) + 1) * (hs + 1));
    }
    default: return 17500000u / (((sys & 0xFF) + 1) * ((cpu & 0xFF) + 1));
    }
}

uint32_t Pcr::timg_hz(int group, bool wdt) const {
    const uint32_t reg = group == 0 ? (wdt ? pcr::TG0_WDT_CLK : pcr::TG0_TIMER_CLK)
                                    : (wdt ? pcr::TG1_WDT_CLK : pcr::TG1_TIMER_CLK);
    switch ((read_const(reg) >> 20) & 3) {
    case 1: return 80000000u;  // PLL_F80M
    case 2: return 17500000u;  // RC_FAST
    default: return 40000000u;  // XTAL
    }
}

void Pcr::write(uint32_t off, uint32_t value) {
    if (before_timg_clock_change && (off == pcr::TG0_TIMER_CLK || off == pcr::TG1_TIMER_CLK))
        before_timg_clock_change(off == pcr::TG0_TIMER_CLK ? 0 : 1);
    if (off == pcr::SYSCLK_CONF)  // LS/HS_DIV_NUM and CLK_XTAL_FREQ are read-only
        value = (value & (3u << 16)) | (read_const(off) & ~(3u << 16));
    RegisterFile::write(off, value);
    if (off == pcr::SYSCLK_CONF || off == pcr::CPU_FREQ_CONF) {
        if (on_clock_change) on_clock_change();
    }
}

// ---------------------------------------------------------------- LP_CLKRST
uint32_t LpClkRst::read(uint32_t off) {
    if (off == 0x10) return reset_reason | (1u << 5);  // RESET_CAUSE, CORE0_RESET_FLAG
    return RegisterFile::read(off);
}

// ---------------------------------------------------------------- LP_AON
void LpAon::write(uint32_t off, uint32_t value) {
    if (off == 0x34 && (value & (1u << 31))) {  // SYS_CFG.HPSYS_SW_RESET
        LOGI("lp_aon: software system reset requested");
        if (on_sw_reset) on_sw_reset(reset::SW_SYS);
        return;
    }
    if (off == 0x38 && (value & (1u << 28))) {  // CPUCORE0_CFG.CPU_CORE0_SW_RESET
        LOGI("lp_aon: software CPU reset requested");
        if (on_sw_reset) on_sw_reset(reset::SW_CPU);
        return;
    }
    RegisterFile::write(off, value);
}

// ---------------------------------------------------------------- LP_WDT
namespace lpwdt {
constexpr uint32_t CONFIG0 = 0x00, CONFIG1 = 0x04, CONFIG4 = 0x10, FEED = 0x14, WPROTECT = 0x18, SWD_CONFIG = 0x1C,
                   SWD_WPROTECT = 0x20, DATE = 0x3FC;
constexpr uint32_t kKey = 0x50D83AA1, kSwdKey = 0x50D83AA1;
}  // namespace lpwdt

void LpWdt::reset() {
    RegisterFile::reset();
    set_default(lpwdt::CONFIG0, (1u << 9) | (1u << 12) | (1u << 13) | (1u << 16) | 20u);
    set_default(lpwdt::CONFIG1, 200000);
    set_default(lpwdt::DATE, 0x2112080);
    update();
}

void LpWdt::write(uint32_t off, uint32_t value) {
    if (off == lpwdt::WPROTECT || off == lpwdt::SWD_WPROTECT) {
        RegisterFile::write(off, value);
        return;
    }
    if (off >= lpwdt::SWD_CONFIG && off < lpwdt::SWD_WPROTECT) {  // super watchdog: not modelled
        if (RegisterFile::read(lpwdt::SWD_WPROTECT) == lpwdt::kSwdKey)
            RegisterFile::write(off, value & ~(1u << 31));  // SWD_FEED self-clears
        return;
    }
    if (off <= lpwdt::FEED) {
        if (RegisterFile::read(lpwdt::WPROTECT) != lpwdt::kKey) return;
        if (off == lpwdt::FEED) {
            if (value & (1u << 31)) update();
            return;
        }
        RegisterFile::write(off, value);
        if (off == lpwdt::CONFIG0 || off == lpwdt::CONFIG1) update();
        return;
    }
    RegisterFile::write(off, value);
}

void LpWdt::update() {
    uint32_t cfg0 = RegisterFile::read(lpwdt::CONFIG0);
    uint32_t action = (cfg0 >> 28) & 7;
    if (!(cfg0 & (1u << 31)) || action < 2) {
        if (on_wdt) on_wdt(-1);
        return;
    }
    double timeout = std::max(RegisterFile::read(lpwdt::CONFIG1), 1u) / kSlowClockHz;
    LOGD("lp_wdt: (re)started, resets in %.0f ms unless fed", timeout * 1000);
    if (on_wdt) on_wdt(timeout);
}

// ---------------------------------------------------------------- LP_TIMER
void LpTimer::write(uint32_t off, uint32_t value) {
    if (off == 0x10 && (value & (1u << 28))) {  // UPDATE.MAIN_TIMER_UPDATE: latch into MAIN_BUF0
        uint64_t t = slow_ticks ? slow_ticks() : 0;
        RegisterFile::write(0x14, uint32_t(t));
        RegisterFile::write(0x18, uint32_t(t >> 32) & 0xFFFF);
        return;
    }
    RegisterFile::write(off, value);
}

// ---------------------------------------------------------------- CACHE
uint32_t Cache::read(uint32_t off) {
    uint32_t v = RegisterFile::read(off);
    switch (off) {
    case 0x2C: return (v & ~(1u << 18)) | ((v >> 16 & 1) << 18);  // FREEZE_CTRL: done follows enable
    case 0x88: return v | (1u << 2);   // LOCK_CTRL: done
    case 0x98: return v | (1u << 4);   // SYNC_CTRL: done
    case 0xD8: return v | (1u << 1);   // PRELOAD_CTRL: done
    case 0x134: return v | (1u << 1);  // AUTOLOAD_CTRL: done
    default: return v;
    }
}

// ---------------------------------------------------------------- SPI0 / MMU
namespace spi0 {
constexpr uint32_t MMU_ITEM_CONTENT = 0x37C, MMU_ITEM_INDEX = 0x380, MMU_POWER_CTRL = 0x384;
}

void Spi0Mmu::reset() {
    RegisterFile::reset();
    index_ = 0;
    table_.fill(0);
    if (on_change) on_change(-1);
}

uint32_t Spi0Mmu::page_size() const {
    switch ((read_const(spi0::MMU_POWER_CTRL) >> 3) & 3) {
    case 1: return 0x8000;
    case 2: return 0x4000;
    default: return 0x10000;  // 3 (8 KiB) is not supported for flash
    }
}

uint32_t Spi0Mmu::read(uint32_t off) {
    if (off == 0x170) return 0xFC000000;  // AXI_ERR_ADDR: all AXI FIFOs empty
    if (off == spi0::MMU_ITEM_CONTENT) return index_ < kEntries ? table_[index_] : 0;
    if (off == spi0::MMU_ITEM_INDEX) return index_;
    return RegisterFile::read(off);
}

void Spi0Mmu::write(uint32_t off, uint32_t value) {
    if (off == spi0::MMU_ITEM_CONTENT) {
        if (index_ < kEntries) set_entry(int(index_), value);
        return;
    }
    if (off == spi0::MMU_ITEM_INDEX) {
        index_ = value;
        return;
    }
    uint32_t old = page_size();
    RegisterFile::write(off, value);
    if (off == spi0::MMU_POWER_CTRL && page_size() != old && on_change) on_change(-1);
}

void Spi0Mmu::set_entry(int index, uint32_t value) {
    value &= 0x7FF;  // page, VALID, SENSITIVE
    if (table_[size_t(index)] == value) return;
    table_[size_t(index)] = value;
    if (on_change) on_change(index);
}

// ---------------------------------------------------------------- I2C_ANA_MST
uint32_t I2cAnaMst::read(uint32_t off) {
    uint32_t v = RegisterFile::read(off);
    if (off == 0x18) return v | (1u << 24);  // ANA_CONF0.BBPLL_CAL_DONE
    return v;
}

void I2cAnaMst::write(uint32_t off, uint32_t value) {
    if (off == 0x00 || off == 0x04) {  // I2Cn_CTRL: slave id [7:0], reg [15:8], data [23:16], write [24]
        uint32_t key = value & 0xFFFF;
        if (value & (1u << 24)) analog_[key] = uint8_t(value >> 16);
        auto it = analog_.find(key);
        uint32_t data = it == analog_.end() ? 0 : it->second;
        if (key == 0x0361) data |= 0x09;  // I2C_ULP reg 3: O_DONE / BG_O_DONE (o-code calibration finished)
        value = (value & 0x0100FFFF) | (data << 16);  // never busy
    }
    RegisterFile::write(off, value);
}

// ---------------------------------------------------------------- INTPRI
void Intpri::write(uint32_t off, uint32_t value) {
    RegisterFile::write(off, value);
    if (off >= 0x90 && off < 0xA0 && on_cpu_intr) on_cpu_intr(int(off - 0x90) / 4, value & 1);
}

}  // namespace esp32sim::c6
