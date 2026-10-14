#include "soc/esp32c3/gpio.hpp"

#include <cstdio>

#include "util/log.hpp"

namespace esp32sim::c3 {

namespace {
constexpr uint32_t OUT = 0x04, OUT_W1TS = 0x08, OUT_W1TC = 0x0C, ENABLE = 0x20, ENABLE_W1TS = 0x24,
                   ENABLE_W1TC = 0x28, STRAP = 0x38, IN = 0x3C, STATUS = 0x44, STATUS_W1TS = 0x48,
                   STATUS_W1TC = 0x4C, PCPU_INT = 0x5C, PCPU_NMI_INT = 0x60, PIN0 = 0x74, STATUS_NEXT = 0x14C,
                   FUNC0_IN_SEL = 0x154, FUNC0_OUT_SEL = 0x554, CLOCK_GATE = 0x62C, DATE = 0x6FC;
constexpr uint32_t kPinMask = (1u << Gpio::kPins) - 1;
constexpr uint32_t SIG_GPIO_OUT = 128;

// IO_MUX_GPIOn fields.
constexpr uint32_t FUN_WPD = 1u << 7, FUN_WPU = 1u << 8, FUN_IE = 1u << 9;
inline uint32_t mcu_sel(uint32_t mux) { return (mux >> 12) & 7; }

// Function 0 of each pad (function 1 is always the GPIO matrix).
const char* const kFunc0[Gpio::kPins] = {"GPIO0", "GPIO1", "GPIO2", "GPIO3", "MTMS",   "MTDI",  "MTCK",  "MTDO",
                                         "GPIO8", "GPIO9", "GPIO10", "VDD_SPI", "SPIHD", "SPIWP", "SPICS0",
                                         "SPICLK", "SPID", "SPIQ", "GPIO18", "GPIO19", "U0RXD", "U0TXD"};
const char* const kFunc2[Gpio::kPins] = {nullptr, nullptr, "FSPIQ", nullptr, "FSPIHD", "FSPIWP", "FSPICLK", "FSPID",
                                         nullptr, nullptr, "FSPICS0"};

uint32_t default_mux(int pin) {
    uint32_t v = 2u << 10;  // FUN_DRV = 2
    bool ie = (pin >= 2 && pin <= 10) || pin == 20 || pin == 21 || (pin >= 12 && pin <= 17);
    if (ie) v |= FUN_IE;
    if (pin == 9) v |= FUN_WPU;  // boot strap pull-up
    if (pin >= 12 && pin <= 17) v |= FUN_WPU;
    return v;
}

const char* signal_name(const SignalName* table, size_t n, uint32_t idx) {
    for (size_t i = 0; i < n; ++i)
        if (uint32_t(table[i].index) == idx) return table[i].name;
    return nullptr;
}

const char* const kIntTypes[8] = {"", "rising", "falling", "any edge", "low level", "high level", "", ""};
}  // namespace

Gpio::Gpio(std::string name) : Peripheral(std::move(name)), iomux_(*this) {
    external_.fill(-1);
    reset();
}

void Gpio::reset() {
    out_ = enable_ = status_ = 0;
    pin_cfg_.fill(0);
    out_sel_.fill(SIG_GPIO_OUT);
    in_sel_.fill(0);
    for (int i = 0; i < kPins; ++i) mux_[size_t(i)] = default_mux(i);
    mux_ctrl_ = 0;
    traced_.fill(-2);
    in_ = 0;
    update();
}

bool Gpio::via_matrix(int pin) const {
    uint32_t sel = mcu_sel(mux_[size_t(pin)]);
    return sel == 1 || (sel == 0 && kFunc0[pin][0] == 'G');
}

int Gpio::chip_drive(int pin, std::string* function) const {
    const uint32_t mux = mux_[size_t(pin)];
    int level = -1;
    std::string fn;
    if (via_matrix(pin)) {
        uint32_t cfg = out_sel_[size_t(pin)];
        uint32_t sig = cfg & 0xFF;
        bool oe, val;
        if (sig == SIG_GPIO_OUT) {
            fn = "GPIO";
            oe = (enable_ >> pin) & 1;
            val = (out_ >> pin) & 1;
        } else {
            const char* n = signal_name(kOutSignals, kOutSignalCount, sig);
            fn = n ? n : "signal " + std::to_string(sig);
            oe = (cfg & (1u << 9)) ? ((enable_ >> pin) & 1) : true;
            // Peripheral outputs are not simulated at the pad; serial TX idles high.
            val = fn == "U0TXD" || fn == "U1TXD";
        }
        if (cfg & (1u << 8)) val = !val;
        if (oe) level = val;
        // Untouched USB pads keep their USB role.
        if ((pin == 18 || pin == 19) && sig == SIG_GPIO_OUT && !oe && mux == default_mux(pin))
            fn = pin == 18 ? "USB D-" : "USB D+";
    } else {
        uint32_t sel = mcu_sel(mux);
        const char* n = sel == 0 ? kFunc0[pin] : sel == 2 ? kFunc2[pin] : nullptr;
        fn = n ? n : "function " + std::to_string(sel);
        if (pin >= 12 && pin <= 17 && sel == 0) fn = "SPI flash (" + fn + ")";
        if (pin == 21 && sel == 0) level = 1;  // U0TXD idles high
    }
    if (level == 1 && (pin_cfg_[size_t(pin)] & (1u << 2))) level = -1;  // open drain releases high
    if (function) *function = fn;
    return level;
}

int Gpio::pad_level(int pin) const {
    int d = chip_drive(pin, nullptr);
    if (d >= 0) return d;
    if (external_[size_t(pin)] >= 0) return external_[size_t(pin)];
    uint32_t mux = mux_[size_t(pin)];
    if (mux & FUN_WPU) return 1;
    if (mux & FUN_WPD) return 0;
    return -1;
}

PinInfo Gpio::info(int pin) const {
    PinInfo p;
    p.gpio = pin;
    if (pin < 0 || pin >= kPins) return p;
    const uint32_t mux = mux_[size_t(pin)];
    p.chip_level = chip_drive(pin, &p.function);
    p.output = p.chip_level >= 0 || (via_matrix(pin) && ((enable_ >> pin) & 1));
    p.input = (mux & FUN_IE) != 0;
    p.pull_up = (mux & FUN_WPU) != 0;
    p.pull_down = (mux & FUN_WPD) != 0;
    p.open_drain = (pin_cfg_[size_t(pin)] & (1u << 2)) != 0;
    p.external = external_[size_t(pin)];
    p.level = pad_level(pin);
    uint32_t cfg = pin_cfg_[size_t(pin)];
    if (cfg & (1u << 13)) p.interrupt = kIntTypes[(cfg >> 7) & 7];
    if (p.function == "GPIO" && !p.output && !p.input) p.function = "GPIO (disabled)";
    return p;
}

void Gpio::set_external(int pin, int level) {
    if (pin < 0 || pin >= kPins) return;
    external_[size_t(pin)] = level < 0 ? -1 : (level ? 1 : 0);
    update();
}

void Gpio::update() {
    uint32_t in = 0;
    for (int i = 0; i < kPins; ++i)
        if ((mux_[size_t(i)] & FUN_IE) && pad_level(i) == 1) in |= 1u << i;
    const uint32_t rose = in & ~in_, fell = ~in & in_;
    in_ = in;
    uint32_t enabled = 0;
    for (int i = 0; i < kPins; ++i) {
        uint32_t cfg = pin_cfg_[size_t(i)];
        if (!(cfg & (1u << 13))) continue;
        enabled |= 1u << i;
        const uint32_t bit = 1u << i;
        switch ((cfg >> 7) & 7) {
        case 1: if (rose & bit) status_ |= bit; break;
        case 2: if (fell & bit) status_ |= bit; break;
        case 3: if ((rose | fell) & bit) status_ |= bit; break;
        case 4: if (!(in & bit)) status_ |= bit; break;
        case 5: if (in & bit) status_ |= bit; break;
        default: break;
        }
    }
    if (trace) {
        for (int i = 0; i < kPins; ++i) {
            int d = chip_drive(i, nullptr);
            if (d != traced_[size_t(i)] && traced_[size_t(i)] != -2 && d >= 0)
                LOGI("gpio: GPIO%d -> %d", i, d);
            traced_[size_t(i)] = d;
        }
    }
    ++version_;
    if (irq) irq((status_ & enabled) != 0);
}

uint32_t Gpio::read(uint32_t off) {
    if (off >= PIN0 && off < PIN0 + 4 * kPins) return pin_cfg_[(off - PIN0) / 4];
    if (off >= FUNC0_OUT_SEL && off < FUNC0_OUT_SEL + 4 * kPins) return out_sel_[(off - FUNC0_OUT_SEL) / 4];
    if (off >= FUNC0_IN_SEL && off < FUNC0_IN_SEL + 4 * 128) return in_sel_[(off - FUNC0_IN_SEL) / 4];
    switch (off) {
    case OUT: return out_;
    case ENABLE: return enable_;
    case STRAP: return strap_;
    case IN: return in_;
    case STATUS: return status_;
    case STATUS_NEXT: return status_;
    case PCPU_INT: {
        uint32_t enabled = 0;
        for (int i = 0; i < kPins; ++i)
            if (pin_cfg_[size_t(i)] & (1u << 13)) enabled |= 1u << i;
        return status_ & enabled;
    }
    case PCPU_NMI_INT: return 0;
    case CLOCK_GATE: return 1;
    case DATE: return 0x2006130;
    default: return 0;
    }
}

void Gpio::write(uint32_t off, uint32_t value) {
    if (off >= PIN0 && off < PIN0 + 4 * kPins) {
        pin_cfg_[(off - PIN0) / 4] = value;
    } else if (off >= FUNC0_OUT_SEL && off < FUNC0_OUT_SEL + 4 * kPins) {
        out_sel_[(off - FUNC0_OUT_SEL) / 4] = value & 0x7FF;
    } else if (off >= FUNC0_IN_SEL && off < FUNC0_IN_SEL + 4 * 128) {
        in_sel_[(off - FUNC0_IN_SEL) / 4] = value & 0x7F;
        return;  // input routing has no visible effect yet
    } else {
        switch (off) {
        case OUT: out_ = value & kPinMask; break;
        case OUT_W1TS: out_ |= value & kPinMask; break;
        case OUT_W1TC: out_ &= ~value; break;
        case ENABLE: enable_ = value & kPinMask; break;
        case ENABLE_W1TS: enable_ |= value & kPinMask; break;
        case ENABLE_W1TC: enable_ &= ~value; break;
        case STATUS: status_ = value & kPinMask; break;
        case STATUS_W1TS: status_ |= value & kPinMask; break;
        case STATUS_W1TC: status_ &= ~value; break;
        default: return;
        }
    }
    update();
}

uint32_t Gpio::IoMux::read(uint32_t off) {
    if (off == 0) return g_.mux_ctrl_;
    if (off >= 4 && off < 4 + 4 * kPins) return g_.mux_[(off - 4) / 4];
    if (off == 0xFC) return 0x2006050;  // DATE
    return 0;
}

void Gpio::IoMux::write(uint32_t off, uint32_t value) {
    if (off == 0) {
        g_.mux_ctrl_ = value;
        return;
    }
    if (off >= 4 && off < 4 + 4 * kPins) {
        g_.mux_[(off - 4) / 4] = value;
        g_.update();
    }
}

}  // namespace esp32sim::c3
