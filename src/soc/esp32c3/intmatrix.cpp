#include "soc/esp32c3/intmatrix.hpp"

#include <algorithm>

#include "util/bits.hpp"
#include "util/log.hpp"

namespace esp32sim::c3 {

namespace {
constexpr uint32_t STATUS0 = 0xF8, STATUS1 = 0xFC, CLOCK_GATE = 0x100, ENABLE = 0x104, TYPE = 0x108;
constexpr uint32_t CLEAR = 0x10C, EIP_STATUS = 0x110, PRI0 = 0x114, THRESH = 0x194, DATE = 0x7FC;
}  // namespace

void IntMatrix::reset() {
    map_.fill(0);
    pri_.fill(0);
    level_[0] = level_[1] = 0;
    edge_latch_ = enable_ = type_ = thresh_ = raw_lines_ = 0;
    clock_gate_ = 1;
    update();
}

uint32_t IntMatrix::read(uint32_t off) {
    if (off < uint32_t(sources_) * 4) return map_[off / 4];
    return inline_ ? cpu_read(off) : 0;
}

void IntMatrix::write(uint32_t off, uint32_t value) {
    if (off < uint32_t(sources_) * 4) {
        map_[off / 4] = uint8_t(value & 0x1F);
        if (value & 0x1F) LOGD("intmatrix: source %u -> CPU line %u", off / 4, value & 0x1F);
        update();
    } else if (inline_) {
        cpu_write(off, value);
    }
}

uint32_t IntMatrix::cpu_read(uint32_t off) {
    if (off >= PRI0 && off < PRI0 + 32 * 4) return pri_[(off - PRI0) / 4];
    switch (off) {
    case STATUS0: return uint32_t(level_[0]);
    case STATUS1: return uint32_t(level_[0] >> 32);
    case CLOCK_GATE: return clock_gate_;
    case ENABLE: return enable_;
    case TYPE: return type_;
    case EIP_STATUS: return raw_lines_;
    case THRESH: return thresh_;
    case DATE: return 0x2007210;
    default: return 0;
    }
}

void IntMatrix::cpu_write(uint32_t off, uint32_t value) {
    if (off >= PRI0 && off < PRI0 + 32 * 4) {
        pri_[(off - PRI0) / 4] = uint8_t(value & 0xF);
    } else {
        switch (off) {
        case CLOCK_GATE: clock_gate_ = value & 1; return;
        case ENABLE: enable_ = value & ~1u; break;
        case TYPE: type_ = value; break;
        case CLEAR: edge_latch_ &= ~value; break;
        case THRESH: thresh_ = value & 0xF; break;
        default: return;
        }
    }
    update();
}

void IntMatrix::set_source(int source, bool level) {
    if (source < 0 || source >= sources_) return;
    uint64_t& word = level_[source >> 6];
    const uint64_t bit = 1ull << (source & 63);
    const bool was = (word & bit) != 0;
    if (was == level) return;
    word = level ? (word | bit) : (word & ~bit);
    if (level && map_[size_t(source)]) {
        uint32_t line = 1u << map_[size_t(source)];
        if (type_ & line) edge_latch_ |= line;
    }
    update();
}

void IntMatrix::update() {
    uint32_t level_lines = 0;
    for (int w = 0; w < 2; ++w) {
        for (uint64_t pending = level_[w]; pending; pending &= pending - 1) {
            int src = w * 64 + ctz64(pending);
            if (map_[size_t(src)]) level_lines |= 1u << map_[size_t(src)];
        }
    }
    raw_lines_ = ((level_lines & ~type_) | (edge_latch_ & type_)) & ~1u;
    // Deliver only the highest-priority enabled lines at or above the
    // threshold. Priority 0 never interrupts.
    uint32_t best = 0;
    int best_pri = std::max(int(thresh_), 1);
    for (uint32_t m = raw_lines_ & enable_; m; m &= m - 1) {
        int line = ctz32(m);
        int p = pri_[size_t(line)];
        if (p < best_pri) continue;
        if (p > best_pri) best_pri = p, best = 0;
        best |= 1u << line;
    }
    if (on_lines) on_lines(best);
}

}  // namespace esp32sim::c3
