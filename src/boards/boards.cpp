#include "boards/boards.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "util/json.hpp"

namespace esp32sim {

namespace fs = std::filesystem;

namespace {
bool read_file(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}
}  // namespace

uint32_t BoardDb::parse_size(const std::string& s) {
    if (s.empty()) return 0;
    char* end = nullptr;
    double v = std::strtod(s.c_str(), &end);
    std::string unit = end ? end : "";
    for (auto& c : unit) c = char(std::toupper(static_cast<unsigned char>(c)));
    if (unit == "MB" || unit == "M") v *= 1024 * 1024;
    else if (unit == "KB" || unit == "K") v *= 1024;
    return uint32_t(v);
}

std::string BoardDb::default_platformio_dir() {
    const char* core = std::getenv("PLATFORMIO_CORE_DIR");
    std::string base;
    if (core) base = core;
    else {
#ifdef _WIN32
        const char* home = std::getenv("USERPROFILE");
#else
        const char* home = std::getenv("HOME");
#endif
        if (!home) return {};
        base = std::string(home) + "/.platformio";
    }
    return base + "/platforms/espressif32/boards";
}

void BoardDb::add(const BoardInfo& b) {
    auto it = std::find_if(boards_.begin(), boards_.end(), [&](const BoardInfo& x) { return x.id == b.id; });
    if (it != boards_.end()) *it = b;
    else boards_.push_back(b);
}

bool BoardDb::load_cache(const std::string& path, std::string* error) {
    std::string text;
    if (!read_file(path, text)) {
        if (error) *error = "cannot read " + path;
        return false;
    }
    Json j;
    if (!Json::parse(text, j, error)) return false;
    for (const auto& e : j["boards"].array()) {
        BoardInfo b;
        b.id = e["id"].str();
        b.name = e["name"].str();
        b.mcu = e["mcu"].str();
        b.vendor = e["vendor"].str();
        b.url = e["url"].str();
        b.f_cpu = uint32_t(e["f_cpu"].num());
        b.flash_size = parse_size(e["flash_size"].str());
        b.ram = uint32_t(e["ram"].num());
        if (!b.id.empty()) add(b);
    }
    source_ = path + " (" + j["source"].str("unknown source") + ", " + j["generated"].str("?") + ")";
    return true;
}

bool BoardDb::load_platformio_dir(const std::string& dir, std::string* error) {
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
        if (error) *error = dir + " is not a directory";
        return false;
    }
    size_t n = 0;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (entry.path().extension() != ".json") continue;
        std::string text;
        Json j;
        if (!read_file(entry.path().string(), text) || !Json::parse(text, j)) continue;
        BoardInfo b;
        b.id = entry.path().stem().string();
        b.name = j["name"].str();
        b.mcu = j["build"]["mcu"].str();
        b.vendor = j["vendor"].str();
        b.url = j["url"].str();
        b.f_cpu = uint32_t(j["build"]["f_cpu"].num());  // "160000000L" parses as a number prefix
        b.flash_size = parse_size(j["upload"]["flash_size"].str());
        b.ram = uint32_t(j["upload"]["maximum_ram_size"].num());
        add(b);
        ++n;
    }
    source_ = dir;
    if (!n && error) *error = "no board manifests in " + dir;
    return n > 0;
}

const BoardInfo* BoardDb::find(const std::string& id) const {
    for (const auto& b : boards_)
        if (b.id == id) return &b;
    return nullptr;
}

}  // namespace esp32sim
