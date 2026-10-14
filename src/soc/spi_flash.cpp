#include "soc/spi_flash.hpp"

#include <algorithm>
#include <cstdio>

#include "util/log.hpp"

namespace esp32sim {

SpiFlash::SpiFlash(uint32_t size_bytes) : data_(size_bytes, 0xFF) {
    // GigaDevice GD25Qxx, capacity code = log2(size).
    uint32_t cap = 0;
    while ((1u << cap) < size_bytes) ++cap;
    jedec_ = 0xC8 | (0x40 << 8) | (cap << 16);
}

bool SpiFlash::load(const std::string& path) {
    path_ = path;
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        LOGI("flash: %s does not exist yet, starting with erased flash (%u KiB)", path.c_str(), size() / 1024);
        return true;
    }
    size_t n = std::fread(data_.data(), 1, data_.size(), f);
    std::fclose(f);
    LOGI("flash: loaded %zu bytes from %s", n, path.c_str());
    return true;
}

bool SpiFlash::save() {
    if (path_.empty() || !dirty_) return true;
    std::string tmp = path_ + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) {
        LOGE("flash: cannot write %s", tmp.c_str());
        return false;
    }
    bool ok = std::fwrite(data_.data(), 1, data_.size(), f) == data_.size();
    ok = (std::fclose(f) == 0) && ok;
    if (ok) {
        std::remove(path_.c_str());
        ok = std::rename(tmp.c_str(), path_.c_str()) == 0;
    }
    if (ok) dirty_ = false;
    else LOGE("flash: failed to save %s", path_.c_str());
    return ok;
}

void SpiFlash::erase(uint32_t addr, uint32_t len) {
    if (addr >= size()) return;
    len = std::min(len, size() - addr);
    std::fill(data_.begin() + addr, data_.begin() + addr + len, 0xFF);
    dirty_ = true;
    if (on_modified) on_modified();
}

void SpiFlash::program(uint32_t addr, const uint8_t* src, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        uint32_t a = addr + i;
        if (a < size()) data_[a] &= src[i];
    }
    dirty_ = true;
    if (on_modified) on_modified();
}

void SpiFlash::read(uint32_t addr, uint8_t* dst, uint32_t len) const {
    for (uint32_t i = 0; i < len; ++i) {
        uint32_t a = addr + i;
        dst[i] = a < size() ? data_[a] : 0xFF;
    }
}

void SpiFlash::transaction(uint8_t cmd, uint32_t addr, bool has_addr, const std::vector<uint8_t>& mosi,
                           std::vector<uint8_t>& miso, size_t miso_len) {
    miso.assign(miso_len, 0xFF);
    auto need_wel = [&](const char* what) {
        if (!(status_ & 2)) {
            LOGD("flash: %s without write enable ignored", what);
            return false;
        }
        status_ &= ~2;
        return true;
    };
    switch (cmd) {
    case 0x9F:  // RDID
        for (size_t i = 0; i < miso_len && i < 3; ++i) miso[i] = uint8_t(jedec_ >> (8 * i));
        break;
    case 0x05: std::fill(miso.begin(), miso.end(), status_); break;
    case 0x35: std::fill(miso.begin(), miso.end(), status2_); break;
    case 0x06: status_ |= 2; break;
    case 0x04: status_ &= ~2; break;
    case 0x01:  // WRSR
        if (need_wel("WRSR") && !mosi.empty()) {
            status_ = mosi[0] & 0xFC;
            if (mosi.size() > 1) status2_ = mosi[1];
        }
        break;
    case 0x03:
    case 0x0B:
    case 0x3B:
    case 0x6B:
    case 0xBB:
    case 0xEB:  // reads (dummy cycles handled by controller)
        if (has_addr) read(addr, miso.data(), uint32_t(miso_len));
        break;
    case 0x02:  // page program, wraps within a 256-byte page
        if (need_wel("PP")) {
            for (size_t i = 0; i < mosi.size(); ++i) {
                uint32_t a = (addr & ~0xFFu) | ((addr + uint32_t(i)) & 0xFF);
                program(a, &mosi[i], 1);
            }
        }
        break;
    case 0x20: if (need_wel("SE")) erase(addr & ~0xFFFu, 0x1000); break;
    case 0x52: if (need_wel("BE32")) erase(addr & ~0x7FFFu, 0x8000); break;
    case 0xD8: if (need_wel("BE64")) erase(addr & ~0xFFFFu, 0x10000); break;
    case 0x60:
    case 0xC7: if (need_wel("CE")) erase(0, size()); break;
    case 0xAB:  // release power down / device id
        std::fill(miso.begin(), miso.end(), uint8_t((jedec_ >> 16) - 1));
        break;
    case 0xB9: break;  // deep power down: nothing to model
    case 0x66: case 0x99: case 0x75: case 0x7A: case 0xB0: case 0x30:
        break;  // reset enable/reset, erase suspend/resume: no internal state to touch
    case 0x5A:  // SFDP: not provided, reads as erased so drivers fall back to JEDEC tables
        std::fill(miso.begin(), miso.end(), uint8_t(0xFF));
        break;
    case 0x4B:  // unique id
        for (size_t i = 0; i < miso_len; ++i) miso[i] = uint8_t(0x10 + i);
        break;
    default:
        LOGD("flash: unhandled SPI command 0x%02x", cmd);
        break;
    }
}

SpiMemController::SpiMemController(std::string name, SpiFlash& flash, SpiMemLayout layout)
    : RegisterFile(std::move(name)), flash_(flash), l_(layout) {}

uint8_t SpiMemController::wbyte(size_t i) {
    return uint8_t(RegisterFile::read(l_.w0 + uint32_t(i / 4) * 4) >> (8 * (i % 4)));
}

void SpiMemController::set_wbyte(size_t i, uint8_t v) {
    uint32_t off = l_.w0 + uint32_t(i / 4) * 4;
    uint32_t w = RegisterFile::read(off);
    int sh = int(8 * (i % 4));
    w = (w & ~(0xFFu << sh)) | (uint32_t(v) << sh);
    RegisterFile::write(off, w);
}

uint32_t SpiMemController::read(uint32_t offset) {
    if (offset == l_.cmd) return 0;  // operations complete instantly
    if (offset == l_.rd_status) return flash_.status();
    return RegisterFile::read(offset);
}

void SpiMemController::write(uint32_t offset, uint32_t value) {
    RegisterFile::write(offset, value);
    if (offset == l_.cmd && value) execute(value);
}

void SpiMemController::execute(uint32_t c) {
    std::vector<uint8_t> mosi, miso;
    const uint32_t addr_reg = RegisterFile::read(l_.addr);
    if (c & (1u << 18)) {  // USR: user-defined transaction
        uint32_t user = RegisterFile::read(l_.user);
        uint32_t user1 = RegisterFile::read(l_.user1);
        uint32_t user2 = RegisterFile::read(l_.user2);
        uint8_t opcode = uint8_t(user2 & 0xFF);
        bool has_addr = user & (1u << 30);
        uint32_t addr_bits = ((user1 >> 26) & 0x3F) + 1;
        uint32_t addr = addr_reg;
        if (has_addr && addr_bits < 32) addr &= (1u << addr_bits) - 1;
        if (user & (1u << 27)) {
            size_t n = (RegisterFile::read(l_.mosi_dlen) + 1 + 7) / 8;
            for (size_t i = 0; i < n && i < 64; ++i) mosi.push_back(wbyte(i));
        }
        size_t miso_len = 0;
        if (user & (1u << 28)) miso_len = std::min<size_t>((RegisterFile::read(l_.miso_dlen) + 1 + 7) / 8, 64);
        flash_.transaction(opcode, addr, has_addr, mosi, miso, miso_len);
        for (size_t i = 0; i < miso.size(); ++i) set_wbyte(i, miso[i]);
        LOGT("spi: USR cmd=0x%02x addr=0x%06x mosi=%zu miso=%zu", opcode, addr, mosi.size(), miso.size());
        return;
    }
    // Pre-canned hardware commands.
    uint32_t addr = addr_reg & 0xFFFFFF;
    uint32_t len = addr_reg >> 24;
    auto simple = [&](uint8_t op) { flash_.transaction(op, addr, true, mosi, miso, 0); };
    if (c & (1u << 30)) simple(0x06);
    if (c & (1u << 29)) simple(0x04);
    if (c & (1u << 28)) {  // RDID
        flash_.transaction(0x9F, 0, false, mosi, miso, 3);
        RegisterFile::write(l_.w0, miso[0] | (miso[1] << 8) | (miso[2] << 16));
    }
    if (c & (1u << 27)) RegisterFile::write(l_.rd_status, flash_.status());
    if (c & (1u << 26)) {  // WRSR
        mosi.push_back(uint8_t(RegisterFile::read(l_.rd_status)));
        flash_.transaction(0x01, 0, false, mosi, miso, 0);
    }
    if (c & (1u << 25)) {  // PP
        for (uint32_t i = 0; i < len && i < 64; ++i) mosi.push_back(wbyte(i));
        flash_.transaction(0x02, addr, true, mosi, miso, 0);
    }
    if (c & (1u << 24)) simple(0x20);
    if (c & (1u << 23)) simple(0xD8);
    if (c & (1u << 22)) simple(0xC7);
    if (c & (1u << 31)) {  // READ
        uint32_t n = len ? len : (RegisterFile::read(l_.miso_dlen) + 1 + 7) / 8;
        flash_.transaction(0x03, addr, true, mosi, miso, std::min<uint32_t>(n, 64));
        for (size_t i = 0; i < miso.size(); ++i) set_wbyte(i, miso[i]);
    }
}

}  // namespace esp32sim
