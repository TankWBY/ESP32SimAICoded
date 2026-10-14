#include "soc/esp32c3/peripherals.hpp"
#include <algorithm>
#include <vector>

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
    case 0x98: return 0;  // REG_UPDATE (C6): configuration synced at once
    default: return RegisterFile::read(off);
    }
}

void Uart::write(uint32_t off, uint32_t value) {
    switch (off) {
    case uart::FIFO:
        if ((RegisterFile::read(uart::CONF0) >> loopback_bit) & 1) {  // TX wired to RX inside the UART
            if (rx_space()) push_rx(uint8_t(value));
        } else if (on_tx) {
            on_tx(uint8_t(value));
        }
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
Efuse::Efuse(std::string name, const std::array<uint8_t, 6>& mac, uint32_t sys3) : RegisterFile(std::move(name)) {
    // BLOCK1 (EFUSE_RD_MAC_SPI_SYS_n) starts at +0x44.
    set_default(0x44, (uint32_t(mac[2]) << 24) | (uint32_t(mac[3]) << 16) | (uint32_t(mac[4]) << 8) | mac[5]);
    set_default(0x48, (uint32_t(mac[0]) << 8) | mac[1]);
    // Word 3: chip revision and package.
    set_default(0x50, sys3);
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
    if (off == rng_off_) {  // WDEV_RND: xorshift64*, never returns the same value twice
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
namespace usbjtag {
constexpr uint32_t EP1 = 0x00, EP1_CONF = 0x04, INT_RAW = 0x08, INT_ST = 0x0C, INT_ENA = 0x10, INT_CLR = 0x14,
                   FRAM_NUM = 0x24, DATE = 0x80;
constexpr uint32_t WR_DONE = 1u << 0, IN_EP_DATA_FREE = 1u << 1, OUT_EP_DATA_AVAIL = 1u << 2;
constexpr uint32_t INT_OUT_RECV_PKT = 1u << 2, INT_IN_EMPTY = 1u << 3;
}  // namespace usbjtag

void UsbSerialJtag::reset() {
    RegisterFile::reset();
    rx_.clear();
    rx_frame_ = ~0u;
    int_raw_ = usbjtag::INT_IN_EMPTY;
    update_irq();
}

void UsbSerialJtag::update_irq() {
    if (irq) irq((int_raw_ & RegisterFile::read(usbjtag::INT_ENA)) != 0);
}

size_t UsbSerialJtag::rx_space() const {
    const uint32_t frame = frame_number ? frame_number() : 0;
    return rx_.empty() && frame != rx_frame_ ? kPacket : 0;
}

void UsbSerialJtag::push_rx(uint8_t b) {
    rx_frame_ = frame_number ? frame_number() : 0;
    rx_.push_back(b);
    int_raw_ |= usbjtag::INT_OUT_RECV_PKT;
    update_irq();
}

uint32_t UsbSerialJtag::read(uint32_t off) {
    using namespace usbjtag;
    switch (off) {
    case EP1: {
        if (rx_.empty()) return 0;
        uint8_t b = rx_.front();
        rx_.pop_front();
        return b;
    }
    case EP1_CONF: return IN_EP_DATA_FREE | (rx_.empty() ? 0 : OUT_EP_DATA_AVAIL);
    case INT_RAW: return int_raw_;
    case INT_ST: return int_raw_ & RegisterFile::read(INT_ENA);
    case FRAM_NUM: return frame_number ? frame_number() & 0x7FF : 0;
    default: return RegisterFile::read(off);
    }
}

void UsbSerialJtag::write(uint32_t off, uint32_t value) {
    using namespace usbjtag;
    switch (off) {
    case EP1:  // byte to the host
        if (on_tx) on_tx(uint8_t(value));
        return;
    case EP1_CONF:
        if (value & WR_DONE) {  // flush: the host takes the packet at once
            int_raw_ |= INT_IN_EMPTY;
            update_irq();
        }
        return;
    case INT_CLR:
        int_raw_ &= ~value;
        update_irq();
        return;
    default:
        RegisterFile::write(off, value);
        if (off == INT_ENA) update_irq();
        return;
    }
}

// ---------------------------------------------------------------- SAR ADC

namespace {
constexpr uint32_t ADC_ONETIME_SAMPLE = 0x20, ADC_DATA1 = 0x2C, ADC_DATA2 = 0x30;
constexpr uint32_t ADC_INT_RAW = 0x44, ADC_INT_ST = 0x48, ADC_INT_CLR = 0x4C, ADC_INT_ENA = 0x40;
constexpr uint32_t ADC1_ONETIME = 1u << 31, ADC2_ONETIME = 1u << 30, ADC_START = 1u << 29;
constexpr uint32_t ADC1_DONE = 1u << 31, ADC2_DONE = 1u << 30;
}  // namespace

uint32_t SarAdc::read(uint32_t off) {
    if (off == ADC_INT_RAW) return int_raw_;
    if (off == ADC_INT_ST) return int_raw_ & RegisterFile::read(ADC_INT_ENA);
    return RegisterFile::read(off);
}

void SarAdc::write(uint32_t off, uint32_t value) {
    if (off == ADC_INT_CLR) {
        int_raw_ &= ~value;
        return;
    }
    if (off == ADC_ONETIME_SAMPLE) {
        const uint32_t old = RegisterFile::read(off);
        RegisterFile::write(off, value);
        if (!(value & ADC_START) || (old & ADC_START)) return;
        // onetime_channel is (unit << 3) | channel; channel 0xF / 7 is internal.
        const uint32_t sel = (value >> 25) & 0xF;
        const int channel = int(sel & 7);
        auto convert = [&](int unit) {
            uint32_t v = channel == 7 || !sample ? 0 : sample(unit, channel) & 0xFFF;
            return v | uint32_t(channel) << 13;  // the channel tag that ADC2 arbitration checks
        };
        if (value & ADC1_ONETIME) {
            RegisterFile::write(ADC_DATA1, convert(0));
            int_raw_ |= ADC1_DONE;
        }
        if (value & ADC2_ONETIME) {
            RegisterFile::write(ADC_DATA2, convert(1));
            int_raw_ |= ADC2_DONE;
        }
        return;
    }
    RegisterFile::write(off, value);
}

void SarAdc::reset() {
    RegisterFile::reset();
    int_raw_ = 0;
}

// ---------------------------------------------------------------- LEDC

namespace ledc {
constexpr uint32_t CH_STRIDE = 0x14, CONF0 = 0x0, DUTY = 0x8, CONF1 = 0xC, DUTY_R = 0x10;
constexpr uint32_t TIMER0_CONF = 0xA0, INT_RAW = 0xC0, INT_ST = 0xC4, INT_ENA = 0xC8, INT_CLR = 0xCC;
constexpr uint32_t PARA_UP = 1u << 4, DUTY_START = 1u << 31, TIMER_PARA_UP = 1u << 26, TIMER_RST = 1u << 24;
}  // namespace ledc

uint32_t Ledc::read(uint32_t off) {
    using namespace ledc;
    if (off == INT_RAW) return int_raw_;
    if (off == INT_ST) return int_raw_ & RegisterFile::read(INT_ENA);
    return RegisterFile::read(off);
}

void Ledc::write(uint32_t off, uint32_t value) {
    using namespace ledc;
    if (off == INT_CLR) {
        int_raw_ &= ~value;
        update_irq();
        return;
    }
    if (off >= TIMER0_CONF && off < TIMER0_CONF + 0x20 && (off & 4) == 0) {
        RegisterFile::write(off, value & ~TIMER_PARA_UP);  // self-clearing; RST stays as written
        return;
    }
    if (off < kChannels * CH_STRIDE) {
        const uint32_t ch = off / CH_STRIDE, base = ch * CH_STRIDE, reg = off % CH_STRIDE;
        if (reg == DUTY_R) return;  // read-only
        const bool start = (reg == CONF0 && (value & PARA_UP)) || (reg == CONF1 && (value & DUTY_START));
        RegisterFile::write(off, reg == CONF0 ? value & ~PARA_UP : reg == CONF1 ? value & ~DUTY_START : value);
        if (start) {
            uint32_t duty = RegisterFile::read(base + DUTY) & 0x7FFFFF;  // 4 fractional bits
            const uint32_t c1 = RegisterFile::read(base + CONF1) | (reg == CONF1 ? value : 0);
            if (conf1_fade_) {
                const uint32_t scale = c1 & 0x3FF, num = (c1 >> 20) & 0x3FF;
                const int64_t step = int64_t(scale) << 4;
                int64_t d = int64_t(duty) + ((c1 >> 30) & 1 ? step : -step) * int64_t(num > 1 ? num : 0);
                duty = uint32_t(d < 0 ? 0 : d);
            }
            RegisterFile::write(base + DUTY_R, duty);
            int_raw_ |= 1u << (4 + ch);  // DUTY_CHNG_END
            update_irq();
        }
        return;
    }
    RegisterFile::write(off, value);
    if (off == INT_ENA) update_irq();
}

void Ledc::reset() {
    RegisterFile::reset();
    int_raw_ = 0;
    update_irq();
}

void Ledc::update_irq() {
    if (irq) irq((int_raw_ & RegisterFile::read(ledc::INT_ENA)) != 0);
}

// ---------------------------------------------------------------- GP-SPI

namespace gpspi {
constexpr uint32_t CMD = 0x00, USER = 0x10, MS_DLEN = 0x1C, DMA_CONF = 0x30, DMA_INT_ENA = 0x34,
                   DMA_INT_CLR = 0x38, DMA_INT_RAW = 0x3C, DMA_INT_ST = 0x40, DMA_INT_SET = 0x44, W0 = 0x98,
                   DATE = 0xF0;
constexpr uint32_t CMD_UPDATE = 1u << 23, CMD_USR = 1u << 24;
constexpr uint32_t USR_MISO = 1u << 28, USR_MOSI = 1u << 27;
constexpr uint32_t DMA_RX_ENA = 1u << 27, DMA_TX_ENA = 1u << 28;
constexpr uint32_t TRANS_DONE = 1u << 12;
}  // namespace gpspi

uint32_t GpSpi::read(uint32_t off) {
    using namespace gpspi;
    switch (off) {
    case DMA_INT_RAW: return int_raw_;
    case DMA_INT_ST: return int_raw_ & RegisterFile::read(DMA_INT_ENA);
    default: return RegisterFile::read(off);
    }
}

void GpSpi::write(uint32_t off, uint32_t value) {
    using namespace gpspi;
    switch (off) {
    case CMD:
        RegisterFile::write(off, value & ~(CMD_UPDATE | CMD_USR));
        if (value & CMD_USR) transfer();
        return;
    case DMA_INT_CLR:
        int_raw_ &= ~value;
        update_irq();
        return;
    case DMA_INT_SET:
        int_raw_ |= value;
        update_irq();
        return;
    default:
        RegisterFile::write(off, value);
        if (off == DMA_INT_ENA) update_irq();
        return;
    }
}

void GpSpi::reset() {
    RegisterFile::reset();
    RegisterFile::set_default(gpspi::DATE, 0x2007220);
    int_raw_ = 0;
    update_irq();
}

void GpSpi::update_irq() {
    if (irq) irq((int_raw_ & RegisterFile::read(gpspi::DMA_INT_ENA)) != 0);
}

void GpSpi::transfer() {
    using namespace gpspi;
    const uint32_t user = RegisterFile::read(USER), dma = RegisterFile::read(DMA_CONF);
    const size_t bits = (RegisterFile::read(MS_DLEN) & 0x3FFFF) + 1;
    const size_t len = (bits + 7) / 8;
    if (user & (USR_MOSI | USR_MISO)) {
        std::vector<uint8_t> out(len, 0xFF), in(len);
        if (user & USR_MOSI) {
            if ((dma & DMA_TX_ENA) && dma_read) {
                dma_read(out.data(), len);
            } else {
                for (size_t i = 0; i < len && i < 64; ++i) out[i] = uint8_t(RegisterFile::read(W0 + uint32_t(i / 4) * 4) >> (8 * (i % 4)));
            }
        }
        for (size_t i = 0; i < len; ++i) in[i] = exchange ? exchange(out[i]) : 0xFF;
        if (user & USR_MISO) {
            if ((dma & DMA_RX_ENA) && dma_write) {
                dma_write(in.data(), len);
            } else {
                for (size_t w = 0; w * 4 < len && w < 16; ++w) {
                    uint32_t v = RegisterFile::read(W0 + uint32_t(w) * 4);
                    for (size_t b = 0; b < 4 && w * 4 + b < len; ++b)
                        v = (v & ~(0xFFu << (8 * b))) | uint32_t(in[w * 4 + b]) << (8 * b);
                    RegisterFile::write(W0 + uint32_t(w) * 4, v);
                }
            }
        }
    }
    int_raw_ |= TRANS_DONE;
    update_irq();
}

}  // namespace esp32sim::c3
