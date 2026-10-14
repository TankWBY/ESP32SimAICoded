// Memory bus used by CPU cores.
//
// Address space is split into 64 KiB pages. A page either points straight at
// host memory (RAM, memory-mapped flash) for fast access, or is null, in which
// case the access goes through the slow MMIO path (peripherals).
#pragma once
#include <cstdint>
#include <cstring>
#include <memory>

namespace esp32sim {

class Bus {
public:
    static constexpr unsigned kPageBits = 16;
    static constexpr uint32_t kPageSize = 1u << kPageBits;
    static constexpr uint32_t kPageMask = kPageSize - 1;

    struct Page {
        uint8_t* ptr = nullptr;  // host memory backing this page, or null for MMIO
        bool writable = false;
    };

    Bus() : pages_(new Page[1u << (32 - kPageBits)]) {}
    virtual ~Bus() = default;

    void map(uint32_t addr, uint8_t* host, uint32_t size, bool writable) {
        for (uint32_t off = 0; off < size; off += kPageSize) {
            Page& p = pages_[(addr + off) >> kPageBits];
            p.ptr = host + off;
            p.writable = writable;
        }
    }
    void unmap(uint32_t addr, uint32_t size) {
        for (uint32_t off = 0; off < size; off += kPageSize)
            pages_[(addr + off) >> kPageBits] = Page{};
    }

    // Slow path, implemented by the SoC (peripherals, unmapped memory).
    virtual uint32_t mmio_read(uint32_t addr, int size) = 0;
    virtual void mmio_write(uint32_t addr, uint32_t value, int size) = 0;

    template <int N>
    inline uint32_t read(uint32_t addr) {
        const Page& p = pages_[addr >> kPageBits];
        uint32_t off = addr & kPageMask;
        if (p.ptr && off <= kPageSize - N) {
            if constexpr (N == 1) return p.ptr[off];
            if constexpr (N == 2) { uint16_t v; std::memcpy(&v, p.ptr + off, 2); return v; }
            if constexpr (N == 4) { uint32_t v; std::memcpy(&v, p.ptr + off, 4); return v; }
        }
        if (p.ptr || (N > 1 && off > kPageSize - N)) {
            // Straddles a page boundary: assemble byte by byte.
            uint32_t v = 0;
            for (int i = 0; i < N; ++i) v |= read<1>(addr + i) << (8 * i);
            return v;
        }
        return mmio_read(addr, N);
    }

    template <int N>
    inline void write(uint32_t addr, uint32_t value) {
        const Page& p = pages_[addr >> kPageBits];
        uint32_t off = addr & kPageMask;
        if (p.ptr && off <= kPageSize - N) {
            if (!p.writable) return;  // writes to ROM / read-only flash mapping are ignored
            if constexpr (N == 1) p.ptr[off] = uint8_t(value);
            if constexpr (N == 2) { uint16_t v = uint16_t(value); std::memcpy(p.ptr + off, &v, 2); }
            if constexpr (N == 4) std::memcpy(p.ptr + off, &value, 4);
            return;
        }
        if (p.ptr || (N > 1 && off > kPageSize - N)) {
            for (int i = 0; i < N; ++i) write<1>(addr + i, value >> (8 * i));
            return;
        }
        mmio_write(addr, value, N);
    }

    uint32_t read32(uint32_t a) { return read<4>(a); }
    void write32(uint32_t a, uint32_t v) { write<4>(a, v); }

private:
    std::unique_ptr<Page[]> pages_;
};

}  // namespace esp32sim
