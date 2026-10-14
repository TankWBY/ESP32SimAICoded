// Memory bus used by CPU cores.
//
// Address space is split into 16 KiB pages (the smallest flash MMU page). A page either points straight at
// host memory (RAM, memory-mapped flash) for fast access, or is null, in which
// case the access goes through the slow MMIO path (peripherals).
//
// Writable pages also carry a code map: one byte per 32-bit word, set while a
// CPU holds translated code for that word. A store that hits a marked word
// calls on_code_write, so the CPU can drop its translations. The map belongs
// to the host memory, so every address alias of the same RAM (the ESP32-C3
// data and instruction buses) shares it.
#pragma once
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace esp32sim {

class Bus {
public:
    static constexpr unsigned kPageBits = 14;
    static constexpr uint32_t kPageSize = 1u << kPageBits;
    static constexpr uint32_t kPageMask = kPageSize - 1;

    struct Page {
        uint8_t* ptr = nullptr;   // host memory backing this page, or null for MMIO
        uint8_t* code = nullptr;  // code map (writable pages only), kPageSize / 4 bytes
        bool writable = false;
    };

    Bus() : pages_(new Page[1u << (32 - kPageBits)]) {}
    virtual ~Bus() = default;

    void map(uint32_t addr, uint8_t* host, uint32_t size, bool writable) {
        for (uint32_t off = 0; off < size; off += kPageSize) {
            Page& p = pages_[(addr + off) >> kPageBits];
            p.ptr = host + off;
            p.writable = writable;
            p.code = writable ? code_map_for(host + off) : nullptr;
        }
    }
    void unmap(uint32_t addr, uint32_t size) {
        for (uint32_t off = 0; off < size; off += kPageSize)
            pages_[(addr + off) >> kPageBits] = Page{};
    }
    const Page& page(uint32_t addr) const { return pages_[addr >> kPageBits]; }

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
            if ((p.code[off >> 2] | p.code[(off + N - 1) >> 2]) && on_code_write) on_code_write();
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

    // Code map maintenance (see the header comment).
    void mark_code(uint32_t addr, uint32_t len) {
        for (uint32_t a = addr & ~3u; a < addr + len; a += 4) {
            const Page& p = pages_[a >> kPageBits];
            if (p.code) p.code[(a & kPageMask) >> 2] = 1;
        }
    }
    void clear_code_marks() {
        for (auto& m : code_maps_) std::memset(m.second.get(), 0, kPageSize / 4);
    }
    std::function<void()> on_code_write;

private:
    uint8_t* code_map_for(uint8_t* host) {
        auto& m = code_maps_[host];
        if (!m) {
            m.reset(new uint8_t[kPageSize / 4]);
            std::memset(m.get(), 0, kPageSize / 4);
        }
        return m.get();
    }

    std::unique_ptr<Page[]> pages_;
    std::map<uint8_t*, std::unique_ptr<uint8_t[]>> code_maps_;
};

}  // namespace esp32sim
