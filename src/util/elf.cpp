#include "util/elf.hpp"

#include <cstring>
#include <fstream>
#include <iterator>

namespace esp32sim {

namespace {
uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    return o + 4 <= b.size() ? uint32_t(b[o]) | (uint32_t(b[o + 1]) << 8) | (uint32_t(b[o + 2]) << 16) | (uint32_t(b[o + 3]) << 24) : 0;
}
uint16_t rd16(const std::vector<uint8_t>& b, size_t o) { return o + 2 <= b.size() ? uint16_t(b[o] | (b[o + 1] << 8)) : 0; }
}  // namespace

bool ElfFile::load(const std::string& path, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { *error = "cannot open " + path; return false; }
    std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (b.size() < 52 || std::memcmp(b.data(), "\x7f" "ELF", 4) != 0 || b[4] != 1 || b[5] != 1) {
        *error = path + " is not a 32-bit little-endian ELF file";
        return false;
    }
    entry_ = rd32(b, 24);
    const uint32_t shoff = rd32(b, 32);
    const uint16_t shentsize = rd16(b, 46), shnum = rd16(b, 48), shstrndx = rd16(b, 50);
    if (!shoff || shentsize < 40 || size_t(shoff) + size_t(shnum) * shentsize > b.size()) {
        *error = path + ": bad section table";
        return false;
    }
    auto sh = [&](int i, int field) { return rd32(b, shoff + size_t(i) * shentsize + field * 4); };
    const uint32_t strtab_off = sh(shstrndx, 4);
    auto name_at = [&](uint32_t base, uint32_t off) {
        size_t p = size_t(base) + off;
        std::string s;
        while (p < b.size() && b[p]) s += char(b[p++]);
        return s;
    };
    sections_.clear();
    symbols_.clear();
    for (int i = 0; i < shnum; ++i) {
        uint32_t type = sh(i, 1), addr = sh(i, 3), off = sh(i, 4), size = sh(i, 5);
        if (type == 2) {  // SYMTAB
            uint32_t link = sh(i, 6);
            uint32_t str = sh(int(link), 4);
            for (uint32_t s = 0; s + 16 <= size; s += 16) {
                size_t e = size_t(off) + s;
                uint32_t value = rd32(b, e + 4);
                uint8_t info = e + 12 < b.size() ? b[e + 12] : 0;
                uint8_t stype = info & 0xF;
                if ((stype == 1 || stype == 2) && value) {  // OBJECT / FUNC
                    std::string n = name_at(str, rd32(b, e));
                    if (!n.empty()) symbols_.emplace(value, n);
                }
            }
            continue;
        }
        if ((type != 1 && type != 8) || !addr || !size) continue;  // PROGBITS / NOBITS with an address
        ElfSection s;
        s.name = name_at(strtab_off, sh(i, 0));
        s.addr = addr;
        s.size = size;
        s.nobits = type == 8;
        if (!s.nobits) {
            if (size_t(off) + size > b.size()) { *error = path + ": truncated section " + s.name; return false; }
            s.data.assign(b.begin() + off, b.begin() + off + size);
        }
        sections_.push_back(std::move(s));
    }
    return true;
}

std::string ElfFile::symbolize(uint32_t addr, uint32_t* off) const {
    auto it = symbols_.upper_bound(addr);
    if (it == symbols_.begin()) return {};
    --it;
    if (off) *off = addr - it->first;
    return it->second;
}

}  // namespace esp32sim
