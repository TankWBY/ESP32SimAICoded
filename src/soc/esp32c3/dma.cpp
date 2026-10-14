#include "soc/esp32c3/dma.hpp"

#include <algorithm>

#include "util/log.hpp"

namespace esp32sim::c3 {
namespace {
// Channel registers, at 0x70 + channel * 0xC0 (same on the C3 and C6).
constexpr uint32_t kChBase = 0x70, kChStride = 0xC0;
constexpr uint32_t IN_CONF0 = 0x00, IN_CONF1 = 0x04, INFIFO_STATUS = 0x08, IN_LINK = 0x10, IN_STATE = 0x14,
                   IN_SUC_EOF_DES = 0x18, IN_DSCR = 0x20, IN_PERI_SEL = 0x30;
constexpr uint32_t OUT_CONF0 = 0x60, OUT_CONF1 = 0x64, OUTFIFO_STATUS = 0x68, OUT_LINK = 0x70, OUT_STATE = 0x74,
                   OUT_EOF_DES = 0x78, OUT_EOF_BFR_DES = 0x7C, OUT_DSCR = 0x80, OUT_PERI_SEL = 0x90;
constexpr uint32_t LINK_ADDR = 0xFFFFF;
constexpr uint32_t IN_START = 1u << 22, IN_STOP = 1u << 21, IN_RESTART = 1u << 23, IN_PARK = 1u << 24;
constexpr uint32_t OUT_START = 1u << 21, OUT_STOP = 1u << 20, OUT_RESTART = 1u << 22, OUT_PARK = 1u << 23;
constexpr uint32_t CONF0_RST = 1u << 0, OUT_AUTO_WRBACK = 1u << 2;
// Descriptor word 0.
constexpr uint32_t D_SIZE = 0xFFF, D_LEN_SHIFT = 12, D_SUC_EOF = 1u << 30, D_OWNER = 1u << 31;
constexpr uint32_t FIFO_EMPTY = 1u << 1;

// Interrupt bits. C3: one register set per channel holding both
// directions. C6: separate RX and TX sets.
enum Event { IN_DONE, IN_SUC_EOF, OUT_DONE, OUT_EOF, OUT_TOTAL_EOF };
constexpr uint32_t kC3Bit[] = {1u << 0, 1u << 1, 1u << 3, 1u << 4, 1u << 8};
constexpr uint32_t kC6Bit[] = {1u << 0, 1u << 1, 1u << 0, 1u << 1, 1u << 3};
}  // namespace

void Gdma::reset() {
    for (auto& c : ch_) c = Channel{};
    misc_ = 0;
    for (int i = 0; i < kChannels; ++i) update_irq(i);
}

int Gdma::find(int periph, bool out) const {
    for (int i = 0; i < kChannels; ++i) {
        const Dir& d = out ? ch_[i].out : ch_[i].in;
        if (int(d.peri_sel & 0x3F) == periph) return i;
    }
    return -1;
}

void Gdma::start(Dir& d) {
    d.cur = d.link & LINK_ADDR ? link_base_ | (d.link & LINK_ADDR) : 0;
    d.offset = 0;
    d.active = d.cur != 0;
}

void Gdma::raise(int ch, bool out, uint32_t event) {
    Channel& c = ch_[ch];
    if (layout_ == Layout::C3) c.in_raw |= kC3Bit[event];
    else (out ? c.out_raw : c.in_raw) |= kC6Bit[event];
    update_irq(ch);
}

void Gdma::update_irq(int ch) {
    if (!irq) return;
    const Channel& c = ch_[ch];
    irq(ch, (c.in_raw & c.in_ena) != 0);
    if (layout_ == Layout::C6) irq(kChannels + ch, (c.out_raw & c.out_ena) != 0);
}

size_t Gdma::out_read(int periph, uint8_t* dst, size_t max, bool* eof) {
    if (eof) *eof = false;
    int ch = find(periph, true);
    if (ch < 0) return 0;
    Dir& d = ch_[ch].out;
    size_t n = 0;
    while (d.active && n < max) {
        const uint32_t dw0 = bus_.read32(d.cur);
        const uint32_t len = (dw0 >> D_LEN_SHIFT) & 0xFFF;
        const uint32_t buf = bus_.read32(d.cur + 4);
        while (d.offset < len && n < max) dst[n++] = uint8_t(bus_.read<1>(buf + d.offset++));
        if (d.offset < len) break;  // caller has enough for now
        // Descriptor finished.
        if (d.conf0 & OUT_AUTO_WRBACK) bus_.write32(d.cur, dw0 & ~D_OWNER);
        d.eof_bfr_des = d.cur;
        raise(ch, true, OUT_DONE);
        const uint32_t next = bus_.read32(d.cur + 8);
        if (dw0 & D_SUC_EOF) {
            d.eof_des = d.cur;
            raise(ch, true, OUT_EOF);
            if (!next) raise(ch, true, OUT_TOTAL_EOF);
            if (eof) *eof = true;
        }
        d.cur = next;
        d.offset = 0;
        d.active = next != 0;
        if (dw0 & D_SUC_EOF) break;
    }
    if (!d.active && n == 0 && eof) *eof = true;
    return n;
}

size_t Gdma::in_write(int periph, const uint8_t* src, size_t len, bool eof) {
    int ch = find(periph, false);
    if (ch < 0) {
        LOGD("gdma: no RX channel bound to peripheral %d", periph);
        return 0;
    }
    Dir& d = ch_[ch].in;
    size_t n = 0;
    while (d.active && (n < len || (eof && n == len))) {
        const uint32_t dw0 = bus_.read32(d.cur);
        const uint32_t size = dw0 & D_SIZE;
        const uint32_t buf = bus_.read32(d.cur + 4);
        while (d.offset < size && n < len) bus_.write<1>(buf + d.offset++, src[n++]);
        const bool last = eof && n == len;
        if (d.offset < size && !last) break;  // room left for the next push
        // Close the descriptor: length, EOF, owner back to the CPU.
        uint32_t w = (dw0 & D_SIZE) | (d.offset << D_LEN_SHIFT);
        if (last) w |= D_SUC_EOF;
        bus_.write32(d.cur, w);
        raise(ch, false, IN_DONE);
        if (last) {
            d.eof_des = d.cur;
            raise(ch, false, IN_SUC_EOF);
        }
        d.cur = bus_.read32(d.cur + 8);
        d.offset = 0;
        d.active = d.cur != 0;
        if (last) break;
    }
    if (n < len) LOGD("gdma: RX link of channel %d full, %zu bytes dropped", ch, len - n);
    return n;
}

uint32_t Gdma::read(uint32_t off) {
    if (off < kChBase) {
        if (layout_ == Layout::C3) {
            if (off >= 0x10 * kChannels) return off == 0x44 ? misc_ : off == 0x48 ? 0x2008250 : 0;
            const Channel& c = ch_[off / 0x10];
            switch (off & 0xF) {
            case 0x0: return c.in_raw;
            case 0x4: return c.in_raw & c.in_ena;
            case 0x8: return c.in_ena;
            default: return 0;
            }
        }
        if (off >= 0x60) return off == 0x64 ? misc_ : off == 0x68 ? 0x2203090 : 0;
        const bool out = off >= 0x30;
        const Channel& c = ch_[((off - (out ? 0x30 : 0)) / 0x10) % kChannels];
        const uint32_t raw = out ? c.out_raw : c.in_raw, ena = out ? c.out_ena : c.in_ena;
        switch (off & 0xF) {
        case 0x0: return raw;
        case 0x4: return raw & ena;
        case 0x8: return ena;
        default: return 0;
        }
    }
    const uint32_t rel = off - kChBase;
    const int ch = int(rel / kChStride);
    if (ch >= kChannels) return 0;
    const Channel& c = ch_[ch];
    switch (rel % kChStride) {
    case IN_CONF0: return c.in.conf0;
    case IN_CONF1: return c.in.conf1;
    case INFIFO_STATUS: return FIFO_EMPTY;
    case IN_LINK: return (c.in.link & ~(IN_START | IN_STOP | IN_RESTART)) | (c.in.active ? 0 : IN_PARK);
    case IN_STATE: return c.in.cur & 0x3FFFF;
    case IN_SUC_EOF_DES: return c.in.eof_des;
    case IN_DSCR: return c.in.cur;
    case IN_PERI_SEL: return c.in.peri_sel;
    case OUT_CONF0: return c.out.conf0;
    case OUT_CONF1: return c.out.conf1;
    case OUTFIFO_STATUS: return FIFO_EMPTY;
    case OUT_LINK: return (c.out.link & ~(OUT_START | OUT_STOP | OUT_RESTART)) | (c.out.active ? 0 : OUT_PARK);
    case OUT_STATE: return c.out.cur & 0x3FFFF;
    case OUT_EOF_DES: return c.out.eof_des;
    case OUT_EOF_BFR_DES: return c.out.eof_bfr_des;
    case OUT_DSCR: return c.out.cur;
    case OUT_PERI_SEL: return c.out.peri_sel;
    default: return 0;
    }
}

void Gdma::write(uint32_t off, uint32_t value) {
    if (off < kChBase) {
        if (layout_ == Layout::C3) {
            if (off >= 0x10 * kChannels) {
                if (off == 0x44) misc_ = value;
                return;
            }
            const int ch = int(off / 0x10);
            Channel& c = ch_[ch];
            if ((off & 0xF) == 0x8) c.in_ena = value;
            else if ((off & 0xF) == 0xC) c.in_raw &= ~value;
            update_irq(ch);
            return;
        }
        if (off >= 0x60) {
            if (off == 0x64) misc_ = value;
            return;
        }
        const bool out = off >= 0x30;
        const int ch = int(((off - (out ? 0x30 : 0)) / 0x10) % kChannels);
        Channel& c = ch_[ch];
        uint32_t& raw = out ? c.out_raw : c.in_raw;
        uint32_t& ena = out ? c.out_ena : c.in_ena;
        if ((off & 0xF) == 0x8) ena = value;
        else if ((off & 0xF) == 0xC) raw &= ~value;
        update_irq(ch);
        return;
    }
    const uint32_t rel = off - kChBase;
    const int ch = int(rel / kChStride);
    if (ch >= kChannels) return;
    Channel& c = ch_[ch];
    switch (rel % kChStride) {
    case IN_CONF0:
        if ((value & CONF0_RST) && !(c.in.conf0 & CONF0_RST)) c.in.active = false, c.in.offset = 0;
        c.in.conf0 = value;
        return;
    case IN_CONF1: c.in.conf1 = value; return;
    case IN_PERI_SEL: c.in.peri_sel = value; return;
    case IN_LINK:
        c.in.link = value;
        if (value & IN_STOP) c.in.active = false;
        if (value & IN_START) start(c.in);
        else if ((value & IN_RESTART) && !c.in.active && c.in.cur) c.in.active = true;
        return;
    case OUT_CONF0:
        if ((value & CONF0_RST) && !(c.out.conf0 & CONF0_RST)) c.out.active = false, c.out.offset = 0;
        c.out.conf0 = value;
        return;
    case OUT_CONF1: c.out.conf1 = value; return;
    case OUT_PERI_SEL: c.out.peri_sel = value; return;
    case OUT_LINK:
        c.out.link = value;
        if (value & OUT_STOP) c.out.active = false;
        if (value & OUT_START) start(c.out);
        else if ((value & OUT_RESTART) && !c.out.active && c.out.cur) c.out.active = true;
        return;
    default: return;
    }
}

}  // namespace esp32sim::c3
