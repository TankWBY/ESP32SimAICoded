#include "soc/esp32c3/timers.hpp"

#include <algorithm>
#include <cmath>

#include "soc/esp32c3/intmatrix.hpp"
#include "util/log.hpp"

namespace esp32sim::c3 {

// ---------------------------------------------------------------- SYSTIMER
namespace systimer {
constexpr uint32_t CONF = 0x00, UNIT0_OP = 0x04, UNIT1_OP = 0x08;
constexpr uint32_t UNIT0_LOAD_HI = 0x0C, UNIT0_LOAD_LO = 0x10, UNIT1_LOAD_HI = 0x14, UNIT1_LOAD_LO = 0x18;
constexpr uint32_t TARGET0_HI = 0x1C, TARGET0_CONF = 0x34;
constexpr uint32_t UNIT0_VALUE_HI = 0x40, UNIT0_VALUE_LO = 0x44, UNIT1_VALUE_HI = 0x48, UNIT1_VALUE_LO = 0x4C;
constexpr uint32_t COMP0_LOAD = 0x50, UNIT0_LOAD = 0x5C, UNIT1_LOAD = 0x60;
constexpr uint32_t INT_ENA = 0x64, INT_RAW = 0x68, INT_CLR = 0x6C, INT_ST = 0x70, DATE = 0xFC;
constexpr uint32_t OP_UPDATE = 1u << 30, OP_VALUE_VALID = 1u << 29;
constexpr uint32_t CONF_PERIOD_MODE = 1u << 30, CONF_UNIT_SEL = 1u << 31, PERIOD_MASK = 0x3FFFFFF;
}  // namespace systimer

void Systimer::reset() {
    RegisterFile::reset();
    set_default(systimer::CONF, 0x46000000);  // clk_en, unit0 work enable
    set_default(systimer::DATE, 0x2006171);
    snapshot_[0] = snapshot_[1] = 0;
    for (auto& t : target_) t = 0;
    int_raw_ = int_ena_ = 0;
    fired_ = 0;
    // Units count from zero at reset.
    offset_[0] = offset_[1] = uint64_t(0) - ticks();
    update_irq();
}

uint32_t Systimer::read(uint32_t off) {
    using namespace systimer;
    switch (off) {
    case UNIT0_OP: case UNIT1_OP: return OP_VALUE_VALID;
    case UNIT0_VALUE_HI: return uint32_t(snapshot_[0] >> 32) & 0xFFFFF;
    case UNIT0_VALUE_LO: return uint32_t(snapshot_[0]);
    case UNIT1_VALUE_HI: return uint32_t(snapshot_[1] >> 32) & 0xFFFFF;
    case UNIT1_VALUE_LO: return uint32_t(snapshot_[1]);
    case INT_ENA: return int_ena_;
    case INT_RAW: return int_raw_;
    case INT_ST: return int_raw_ & int_ena_;
    default: return RegisterFile::read(off);
    }
}

void Systimer::write(uint32_t off, uint32_t value) {
    using namespace systimer;
    switch (off) {
    case UNIT0_OP:
    case UNIT1_OP:
        if (value & OP_UPDATE) snapshot_[off == UNIT0_OP ? 0 : 1] = unit_value(off == UNIT0_OP ? 0 : 1);
        return;
    case UNIT0_LOAD:
    case UNIT1_LOAD: {
        int u = off == UNIT0_LOAD ? 0 : 1;
        uint64_t v = (uint64_t(RegisterFile::read(u ? UNIT1_LOAD_HI : UNIT0_LOAD_HI) & 0xFFFFF) << 32) |
                     RegisterFile::read(u ? UNIT1_LOAD_LO : UNIT0_LOAD_LO);
        offset_[u] = v - ticks();
        return;
    }
    case COMP0_LOAD: case COMP0_LOAD + 4: case COMP0_LOAD + 8: {
        int c = int(off - COMP0_LOAD) / 4;
        uint32_t conf = RegisterFile::read(TARGET0_CONF + 4 * uint32_t(c));
        int u = (conf & CONF_UNIT_SEL) ? 1 : 0;
        if (conf & CONF_PERIOD_MODE)
            target_[c] = unit_value(u) + (conf & PERIOD_MASK);
        else
            target_[c] = (uint64_t(RegisterFile::read(TARGET0_HI + 8 * uint32_t(c)) & 0xFFFFF) << 32) |
                         RegisterFile::read(TARGET0_HI + 4 + 8 * uint32_t(c));
        fired_ &= ~(1u << c);
        tick();
        return;
    }
    case INT_ENA: int_ena_ = value & 7; update_irq(); return;
    case INT_RAW: int_raw_ = value & 7; update_irq(); return;
    case INT_CLR: int_raw_ &= ~value; update_irq(); return;
    case CONF:
        RegisterFile::write(off, value);
        tick();
        return;
    default: break;
    }
    RegisterFile::write(off, value);
}

uint64_t Systimer::next_event() const {
    using namespace systimer;
    uint32_t conf = RegisterFile::read_const(CONF);
    uint64_t best = kNever;
    for (int c = 0; c < 3; ++c) {
        if (!(conf & (1u << (24 - c)))) continue;
        uint32_t tconf = RegisterFile::read_const(TARGET0_CONF + 4 * uint32_t(c));
        bool periodic = (tconf & CONF_PERIOD_MODE) != 0;
        if (!periodic && (fired_ & (1u << c))) continue;
        int u = (tconf & CONF_UNIT_SEL) ? 1 : 0;
        uint64_t due_ticks = target_[c] - offset_[u];  // raw tick count at which the unit reaches target
        uint64_t due = due_ticks <= ticks() ? *clk_.now : due_ticks * div();
        best = std::min(best, due);
    }
    return best;
}

void Systimer::tick() {
    using namespace systimer;
    uint32_t conf = RegisterFile::read(CONF);
    for (int c = 0; c < 3; ++c) {
        if (!(conf & (1u << (24 - c)))) continue;
        uint32_t tconf = RegisterFile::read(TARGET0_CONF + 4 * uint32_t(c));
        bool periodic = (tconf & CONF_PERIOD_MODE) != 0;
        if (!periodic && (fired_ & (1u << c))) continue;
        uint64_t v = unit_value((tconf & CONF_UNIT_SEL) ? 1 : 0);
        if (v < target_[c]) continue;
        int_raw_ |= 1u << c;
        if (periodic) {
            uint64_t period = std::max<uint64_t>(tconf & PERIOD_MASK, 1);
            target_[c] += ((v - target_[c]) / period + 1) * period;
        } else {
            fired_ |= 1u << c;
        }
    }
    update_irq();
}

void Systimer::update_irq() {
    if (!irq) return;
    uint32_t st = int_raw_ & int_ena_;
    for (int c = 0; c < 3; ++c) irq(c, (st >> c) & 1);  // comparator index; the chip maps it to its source
}

// ---------------------------------------------------------------- TIMG
namespace timg {
constexpr uint32_t T0CONFIG = 0x00, T0LO = 0x04, T0HI = 0x08, T0UPDATE = 0x0C, T0ALARMLO = 0x10, T0ALARMHI = 0x14;
constexpr uint32_t T0LOADLO = 0x18, T0LOADHI = 0x1C, T0LOAD = 0x20;
constexpr uint32_t WDTCONFIG0 = 0x48, WDTCONFIG1 = 0x4C, WDTCONFIG2 = 0x50, WDTFEED = 0x60, WDTWPROTECT = 0x64;
constexpr uint32_t RTCCALICFG = 0x68, RTCCALICFG1 = 0x6C, INT_ENA = 0x70, INT_RAW = 0x74, INT_ST = 0x78, INT_CLR = 0x7C;
constexpr uint32_t RTCCALICFG2 = 0x80, DATE = 0xF8, REGCLK = 0xFC;
constexpr uint32_t T0_EN = 1u << 31, T0_INCREASE = 1u << 30, T0_AUTORELOAD = 1u << 29, T0_ALARM_EN = 1u << 10;
constexpr uint32_t T0_USE_XTAL = 1u << 9;
constexpr uint32_t WDT_EN = 1u << 31, WDT_USE_XTAL = 1u << 21, WKEY = 0x50D83AA1;
constexpr uint32_t CALI_START = 1u << 31, CALI_RDY = 1u << 15, CALI_START_CYCLING = 1u << 12;
constexpr uint64_t T0_MASK = (1ull << 54) - 1;
}  // namespace timg

void Timg::reset() {
    using namespace timg;
    RegisterFile::reset();
    set_default(T0CONFIG, 0x60002000);  // increase, autoreload, divider 1
    set_default(WDTCONFIG0, 0x0004C000);  // flashboot mode, reset lengths
    set_default(WDTCONFIG1, 1u << 16);
    set_default(WDTCONFIG2, 26000000);
    set_default(WDTCONFIG2 + 4, 0x7FFFFFF);
    set_default(WDTCONFIG2 + 8, 0xFFFFF);
    set_default(WDTCONFIG2 + 12, 0xFFFFF);
    set_default(WDTWPROTECT, WKEY);
    set_default(RTCCALICFG, 0x13000);  // max 1, start cycling, clk sel RC_SLOW
    set_default(DATE, 0x2006191);
    set_default(REGCLK, 1u << 31);
    t0_base_ = 0;
    t0_mark_ = *clk_.now;
    wdt_stage_ = 0;
    wdt_start_ = *clk_.now;
    int_raw_ = int_ena_ = 0;
    update_irq();
}

double Timg::t0_rate() const {
    using namespace timg;
    uint32_t cfg = RegisterFile::read_const(T0CONFIG);
    if (!(cfg & T0_EN)) return 0;
    uint32_t div = (cfg >> 13) & 0xFFFF;
    if (div == 0) div = 65536;
    double src = timer_clock_hz ? timer_clock_hz() : (cfg & T0_USE_XTAL) ? Clocks::kXtalHz : clk_.apb_hz;
    return src / div / clk_.master_hz;
}

uint64_t Timg::t0_value() const {
    using namespace timg;
    double delta = double(*clk_.now - t0_mark_) * t0_rate();
    bool up = (RegisterFile::read_const(T0CONFIG) & T0_INCREASE) != 0;
    uint64_t d = uint64_t(delta);
    return (up ? t0_base_ + d : t0_base_ - d) & T0_MASK;
}

void Timg::t0_settle() {
    t0_base_ = t0_value();
    t0_mark_ = *clk_.now;
}

uint64_t Timg::wdt_stage_end() const {
    using namespace timg;
    uint32_t cfg = RegisterFile::read_const(WDTCONFIG0);
    if (!(cfg & WDT_EN)) return kNever;
    uint32_t prescale = RegisterFile::read_const(WDTCONFIG1) >> 16;
    if (prescale == 0) prescale = 65536;
    double src = wdt_clock_hz ? wdt_clock_hz() : (cfg & WDT_USE_XTAL) ? Clocks::kXtalHz : clk_.apb_hz;
    double hold = RegisterFile::read_const(WDTCONFIG2 + 4 * uint32_t(wdt_stage_));
    return wdt_start_ + uint64_t(hold * prescale / src * clk_.master_hz);
}

void Timg::wdt_restart() {
    wdt_stage_ = 0;
    wdt_start_ = *clk_.now;
}

uint32_t Timg::read(uint32_t off) {
    using namespace timg;
    switch (off) {
    case T0UPDATE: return 0;
    case INT_ENA: return int_ena_;
    case INT_RAW: return int_raw_;
    case INT_ST: return int_raw_ & int_ena_;
    case RTCCALICFG2: return RegisterFile::read(off) & ~1u;  // never times out
    case RTCCALICFG: {
        uint32_t v = RegisterFile::read(off);
        return (v & CALI_START_CYCLING) ? (v | CALI_RDY) : v;  // cycling mode always has a result
    }
    default: return RegisterFile::read(off);
    }
}

void Timg::write(uint32_t off, uint32_t value) {
    using namespace timg;
    if (off >= WDTCONFIG0 && off <= WDTFEED && RegisterFile::read(WDTWPROTECT) != WKEY) return;
    switch (off) {
    case T0CONFIG:
        t0_settle();
        RegisterFile::write(off, value);
        tick();
        return;
    case T0UPDATE: {
        uint64_t v = t0_value();
        RegisterFile::write(T0LO, uint32_t(v));
        RegisterFile::write(T0HI, uint32_t(v >> 32));
        return;
    }
    case T0LOAD:
        t0_base_ = (uint64_t(RegisterFile::read(T0LOADHI) & 0x3FFFFF) << 32) | RegisterFile::read(T0LOADLO);
        t0_mark_ = *clk_.now;
        return;
    case T0ALARMLO: case T0ALARMHI:
        RegisterFile::write(off, value);
        tick();
        return;
    case WDTCONFIG0: {
        bool was_on = RegisterFile::read(off) & WDT_EN;
        RegisterFile::write(off, value);
        if (!was_on && (value & WDT_EN)) wdt_restart();
        return;
    }
    case WDTFEED: wdt_restart(); return;
    case RTCCALICFG: {
        uint32_t old = RegisterFile::read(off);
        value &= ~CALI_RDY;
        if ((value & CALI_START) && !(old & CALI_START)) {
            // Calibration completes instantly: report how many XTAL cycles
            // `max` slow-clock cycles take.
            uint32_t max = (value >> 16) & 0x7FFF;
            static const double slow_hz[4] = {136000.0, 17500000.0 / 256, 32768.0, 136000.0};
            double cycles = max * double(Clocks::kXtalHz) / slow_hz[(value >> 13) & 3];
            RegisterFile::write(RTCCALICFG1, (uint32_t(cycles) << 7) | 1);
            value |= CALI_RDY;
        } else if (value & CALI_START) {
            value |= old & CALI_RDY;
        }
        RegisterFile::write(off, value);
        return;
    }
    case INT_ENA: int_ena_ = value & 3; update_irq(); return;
    case INT_CLR: int_raw_ &= ~value; update_irq(); return;
    default: break;
    }
    RegisterFile::write(off, value);
}

uint64_t Timg::next_event() const {
    using namespace timg;
    uint64_t best = wdt_stage_end();
    uint32_t cfg = RegisterFile::read_const(T0CONFIG);
    double rate = t0_rate();
    if ((cfg & T0_ALARM_EN) && rate > 0) {
        uint64_t alarm = (uint64_t(RegisterFile::read_const(T0ALARMHI) & 0x3FFFFF) << 32) |
                         RegisterFile::read_const(T0ALARMLO);
        uint64_t v = t0_value();
        bool up = (cfg & T0_INCREASE) != 0;
        uint64_t due;
        if (up ? v >= alarm : v <= alarm) due = *clk_.now;
        else due = *clk_.now + uint64_t(std::ceil(double(up ? alarm - v : v - alarm) / rate));
        best = std::min(best, due);
    }
    return best;
}

void Timg::tick() {
    using namespace timg;
    const uint64_t now = *clk_.now;
    uint32_t cfg = RegisterFile::read(T0CONFIG);
    if ((cfg & T0_ALARM_EN) && (cfg & T0_EN)) {
        uint64_t alarm = (uint64_t(RegisterFile::read(T0ALARMHI) & 0x3FFFFF) << 32) | RegisterFile::read(T0ALARMLO);
        uint64_t v = t0_value();
        bool up = (cfg & T0_INCREASE) != 0;
        if (up ? v >= alarm : v <= alarm) {
            int_raw_ |= 1;
            RegisterFile::write(T0CONFIG, cfg & ~T0_ALARM_EN);  // hardware clears alarm_en
            if (cfg & T0_AUTORELOAD) {
                t0_base_ = (uint64_t(RegisterFile::read(T0LOADHI) & 0x3FFFFF) << 32) | RegisterFile::read(T0LOADLO);
                t0_mark_ = now;
            }
        }
    }
    for (int guard = 0; guard < 4; ++guard) {
        uint64_t end = wdt_stage_end();
        if (end > now) break;
        uint32_t action = (RegisterFile::read(WDTCONFIG0) >> (29 - 2 * wdt_stage_)) & 3;
        wdt_stage_ = (wdt_stage_ + 1) & 3;
        wdt_start_ = end;
        if (action == 1) {
            int_raw_ |= 2;
        } else if (action >= 2) {
            LOGI("%s: watchdog expired (%s reset)", name().c_str(), action == 2 ? "CPU" : "system");
            wdt_restart();
            if (on_wdt_reset) on_wdt_reset(action == 3);
            return;
        }
    }
    update_irq();
}

void Timg::update_irq() {
    if (!irq) return;
    uint32_t st = int_raw_ & int_ena_;
    irq(t0_src_, st & 1);
    irq(wdt_src_, (st >> 1) & 1);
}

}  // namespace esp32sim::c3
