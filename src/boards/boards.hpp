// Board catalogue sourced from PlatformIO's platform-espressif32 manifests.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace esp32sim {

struct BoardInfo {
    std::string id;        // PlatformIO board id, e.g. "esp32-c3-devkitm-1"
    std::string name;      // "Espressif ESP32-C3-DevKitM-1"
    std::string mcu;       // "esp32c3"
    std::string vendor;
    std::string url;
    uint32_t f_cpu = 0;    // Hz
    uint32_t flash_size = 0;  // bytes
    uint32_t ram = 0;      // bytes
};

class BoardDb {
public:
    // Compact cache written by tools/update_boards.py.
    bool load_cache(const std::string& path, std::string* error);
    // A directory of raw PlatformIO board manifests, e.g.
    // ~/.platformio/platforms/espressif32/boards
    bool load_platformio_dir(const std::string& dir, std::string* error);

    const BoardInfo* find(const std::string& id) const;
    const std::vector<BoardInfo>& boards() const { return boards_; }
    const std::string& source() const { return source_; }

    static uint32_t parse_size(const std::string& s);  // "4MB" -> bytes
    static std::string default_platformio_dir();

private:
    void add(const BoardInfo& b);
    std::vector<BoardInfo> boards_;
    std::string source_;
};

}  // namespace esp32sim
