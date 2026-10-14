// Minimal ELF32 little-endian reader: loadable sections and symbols.
#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace esp32sim {

struct ElfSection {
    std::string name;
    uint32_t addr = 0;
    uint32_t size = 0;
    bool nobits = false;           // .bss-like: zero-filled
    std::vector<uint8_t> data;     // empty for nobits
};

class ElfFile {
public:
    bool load(const std::string& path, std::string* error);
    const std::vector<ElfSection>& sections() const { return sections_; }
    uint32_t entry() const { return entry_; }
    // Function/object symbols by address (first name wins).
    const std::map<uint32_t, std::string>& symbols() const { return symbols_; }
    // Nearest symbol at or below addr, "" if none; offset returned in *off.
    std::string symbolize(uint32_t addr, uint32_t* off = nullptr) const;

private:
    std::vector<ElfSection> sections_;
    std::map<uint32_t, std::string> symbols_;
    uint32_t entry_ = 0;
};

}  // namespace esp32sim
