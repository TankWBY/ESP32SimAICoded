#include "soc/esp32c3/rom_hle.hpp"

#include <cstdio>
#include <cstring>

#include "soc/esp32c3/esp32c3.hpp"
#include "util/inflate.hpp"
#include "util/log.hpp"
#include "util/md5.hpp"

namespace esp32sim {

namespace {

enum Op : uint8_t {
    FLASH_BEGIN = 0x02, FLASH_DATA = 0x03, FLASH_END = 0x04,
    MEM_BEGIN = 0x05, MEM_END = 0x06, MEM_DATA = 0x07,
    SYNC = 0x08, WRITE_REG = 0x09, READ_REG = 0x0A,
    SPI_SET_PARAMS = 0x0B, SPI_ATTACH = 0x0D, READ_FLASH_SLOW = 0x0E, CHANGE_BAUDRATE = 0x0F,
    FLASH_DEFL_BEGIN = 0x10, FLASH_DEFL_DATA = 0x11, FLASH_DEFL_END = 0x12,
    SPI_FLASH_MD5 = 0x13, GET_SECURITY_INFO = 0x14,
};

// ROM loader error codes (second status byte).
constexpr uint8_t ERR_INVALID_MSG = 0x05, ERR_FAILED = 0x06, ERR_BAD_CRC = 0x07, ERR_FLASH_WRITE = 0x08,
                  ERR_FLASH_READ = 0x09, ERR_DEFLATE = 0x0B;

constexpr uint32_t kImageChipId = 5;  // ESP32-C3
constexpr uint32_t kDirectBootMagic = 0xAEDB041D;

uint32_t le32(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }
void put32(std::vector<uint8_t>& v, uint32_t x) { for (int i = 0; i < 4; ++i) v.push_back(uint8_t(x >> (8 * i))); }

uint8_t checksum(const uint8_t* p, size_t n) {
    uint8_t c = 0xEF;
    for (size_t i = 0; i < n; ++i) c ^= p[i];
    return c;
}

const char* reset_reason_name(uint32_t r) {
    switch (r) {
    case 1: return "POWERON";
    case 3: return "RTC_SW_SYS_RST";
    case 7: return "TG0WDT_SYS_RST";
    case 8: return "TG1WDT_SYS_RST";
    case 9: return "RTCWDT_SYS_RST";
    case 12: return "RTC_SW_CPU_RST";
    case 16: return "RTCWDT_RTC_RST";
    default: return "UNKNOWN";
    }
}

bool in_range(uint32_t a, uint32_t lo, uint32_t hi) { return a >= lo && a < hi; }

}  // namespace

// ------------------------------------------------------------------ reset

void Esp32c3Rom::on_reset(BootMode mode) {
    using namespace c3map;
    download_mode_ = mode == BootMode::Download;
    in_frame_ = escape_ = false;
    frame_.clear();
    defl_active_ = false;
    defl_buf_.clear();

    char line[128];
    soc_.console_write("ESP-ROM:esp32c3-api1-20210207\r\nBuild:Feb  7 2021\r\n");
    uint32_t r = soc_.reset_reason();
    if (download_mode_) {
        std::snprintf(line, sizeof line, "rst:0x%x (%s),boot:0x4 (DOWNLOAD(USB/UART0/1))\r\n", r, reset_reason_name(r));
        soc_.console_write(line);
        soc_.console_write("waiting for download\r\n");
        LOGI("rom: download mode, waiting for esptool");
        return;
    }
    std::snprintf(line, sizeof line, "rst:0x%x (%s),boot:0xc (SPI_FAST_FLASH_BOOT)\r\n", r, reset_reason_name(r));
    soc_.console_write(line);
    boot_flash();
}

// ------------------------------------------------------------------ flash boot

bool Esp32c3Rom::parse_image(uint32_t offset, Image& img, std::string* error) {
    SpiFlash& fl = soc_.flash();
    uint8_t hdr[24];
    fl.read(offset, hdr, sizeof hdr);
    char buf[96];
    if (hdr[0] != 0xE9) {
        std::snprintf(buf, sizeof buf, "invalid header: 0x%08x", le32(hdr));
        *error = buf;
        return false;
    }
    img.segments.clear();
    img.spi_mode = hdr[2];
    img.spi_speed_size = hdr[3];
    img.entry = le32(hdr + 4);
    img.chip_id = uint16_t(hdr[12] | (hdr[13] << 8));
    if (img.chip_id != kImageChipId) {
        std::snprintf(buf, sizeof buf, "image is for chip id %u, not ESP32-C3 (%u)", img.chip_id, kImageChipId);
        *error = buf;
        return false;
    }
    uint32_t pos = offset + 24;
    uint8_t csum = 0xEF;
    std::vector<uint8_t> tmp;
    for (int i = 0; i < hdr[1]; ++i) {
        uint8_t sh[8];
        fl.read(pos, sh, 8);
        Segment s{le32(sh), le32(sh + 4), pos + 8};
        if (s.len > 16 * 1024 * 1024 || s.flash_offset + s.len > fl.size()) {
            *error = "segment exceeds flash size";
            return false;
        }
        tmp.resize(s.len);
        fl.read(s.flash_offset, tmp.data(), s.len);
        for (uint8_t b : tmp) csum ^= b;
        img.segments.push_back(s);
        pos = s.flash_offset + s.len;
    }
    // Checksum sits in the last byte of the 16-byte aligned padding.
    uint32_t csum_pos = ((pos - offset + 1 + 15) & ~15u) - 1 + offset;
    uint8_t stored = 0;
    fl.read(csum_pos, &stored, 1);
    if (stored != csum) {
        std::snprintf(buf, sizeof buf, "checksum failure. Calculated 0x%02x stored 0x%02x", csum, stored);
        *error = buf;
        return false;
    }
    return true;
}

bool Esp32c3Rom::find_app_partition(uint32_t* offset, std::string* label) {
    SpiFlash& fl = soc_.flash();
    const uint32_t table = 0x8000;
    uint32_t found_ota0 = 0;
    std::string ota0_label;
    for (uint32_t i = 0; i < 95; ++i) {
        uint8_t e[32];
        fl.read(table + i * 32, e, 32);
        if (e[0] != 0xAA || e[1] != 0x50) break;
        uint8_t type = e[2], subtype = e[3];
        uint32_t off = le32(e + 4);
        std::string name(reinterpret_cast<char*>(e + 12), strnlen(reinterpret_cast<char*>(e + 12), 16));
        if (type == 0 && subtype == 0x00) {
            *offset = off;
            *label = name;
            return true;
        }
        if (type == 0 && subtype == 0x10 && !found_ota0) {
            found_ota0 = off;
            ota0_label = name;
        }
    }
    if (found_ota0) {
        *offset = found_ota0;
        *label = ota0_label;
        return true;
    }
    return false;
}

void Esp32c3Rom::boot_flash() {
    using namespace c3map;
    SpiFlash& fl = soc_.flash();
    uint8_t head[8];
    fl.read(0, head, 8);

    // ROM "direct boot": flash starts with the magic word twice; the ROM maps
    // flash 1:1 into the cache windows and jumps to IROM + 8.
    if (le32(head) == kDirectBootMagic && le32(head + 4) == kDirectBootMagic) {
        uint32_t pages = fl.size() / Bus::kPageSize;
        if (pages > c3::Mmu::kEntries) pages = c3::Mmu::kEntries;
        for (uint32_t i = 0; i < pages; ++i) soc_.mmu().set_entry(int(i), i);
        soc_.console_write("SPIWP:0xee\r\nmode:DIO, clock div:1\r\n");
        LOGI("rom: direct boot");
        soc_.start_cpu(IROM_BASE + 8);
        return;
    }

    std::string err;
    Image boot_img;
    if (!parse_image(0, boot_img, &err)) {
        soc_.console_write(err + "\r\n");
        soc_.halt("ROM boot failed: " + err + " (flash at 0x0 holds no bootable image)");
        return;
    }

    // A standard ESP-IDF / Arduino layout has the 2nd stage bootloader at 0x0
    // and the application in a partition. The real 2nd stage bootloader relies
    // on mask-ROM routines that are not emulated yet, so we perform its job
    // (pick the app partition, map flash, jump) at a high level.
    bool rom_stage_only_ram = true;
    for (const auto& s : boot_img.segments)
        if (in_range(s.addr, IROM_BASE, IROM_BASE + CACHE_WINDOW) || in_range(s.addr, DROM_BASE, DROM_BASE + CACHE_WINDOW))
            rom_stage_only_ram = false;
    uint32_t app_off = 0;
    std::string label;
    if (rom_stage_only_ram && !soc_.has_real_rom() && find_app_partition(&app_off, &label)) {
        Image app;
        if (parse_image(app_off, app, &err)) {
            char line[160];
            soc_.console_write("SPIWP:0xee\r\nmode:DIO, clock div:1\r\n");
            std::snprintf(line, sizeof line,
                          "[esp32sim] 2nd stage bootloader emulated: booting partition '%s' at 0x%x\r\n",
                          label.c_str(), app_off);
            soc_.console_write(line);
            LOGI("rom: HLE bootloader -> app '%s' @0x%x, entry 0x%08x", label.c_str(), app_off, app.entry);
            load_and_run(app, false);
            return;
        }
        LOGW("rom: app partition '%s' at 0x%x is not bootable (%s), falling back to image at 0x0", label.c_str(), app_off, err.c_str());
    }
    load_and_run(boot_img, true);
}

void Esp32c3Rom::load_and_run(const Image& img, bool print_rom_log) {
    using namespace c3map;
    static const char* modes[] = {"QIO", "QOUT", "DIO", "DOUT", "FAST_READ", "SLOW_READ"};
    char line[96];
    if (print_rom_log) {
        soc_.console_write("SPIWP:0xee\r\n");
        std::snprintf(line, sizeof line, "mode:%s, clock div:%d\r\n", img.spi_mode < 6 ? modes[img.spi_mode] : "?",
                      (img.spi_speed_size & 0xF) == 0xF ? 1 : 2);
        soc_.console_write(line);
    }
    SpiFlash& fl = soc_.flash();
    std::vector<uint8_t> buf;
    for (const auto& s : img.segments) {
        bool irom = in_range(s.addr, IROM_BASE, IROM_BASE + CACHE_WINDOW);
        bool drom = in_range(s.addr, DROM_BASE, DROM_BASE + CACHE_WINDOW);
        if (irom || drom) {
            // Flash-mapped: point the MMU entries at the segment's flash pages.
            if ((s.addr & 0xFFFF) != (s.flash_offset & 0xFFFF))
                LOGW("rom: segment 0x%08x not 64 KiB-aligned with its flash offset 0x%x", s.addr, s.flash_offset);
            uint32_t first = (s.addr & (CACHE_WINDOW - 1)) >> 16;
            uint32_t last = ((s.addr + (s.len ? s.len - 1 : 0)) & (CACHE_WINDOW - 1)) >> 16;
            uint32_t page = s.flash_offset >> 16;
            for (uint32_t i = first; i <= last; ++i) soc_.mmu().set_entry(int(i), page + (i - first));
            LOGD("rom: map %s 0x%08x len 0x%x -> flash 0x%x", irom ? "IROM" : "DROM", s.addr, s.len, s.flash_offset);
            continue;
        }
        bool ram = in_range(s.addr, SRAM0_BASE, SRAM_END_IRAM) || in_range(s.addr, DRAM_BASE, DRAM_BASE + DRAM_SIZE) ||
                   in_range(s.addr, RTC_FAST_BASE, RTC_FAST_BASE + RTC_FAST_SIZE);
        if (!ram) {
            if (s.addr == 0) continue;  // elf2image alignment padding
            LOGW("rom: skipping segment at unsupported address 0x%08x", s.addr);
            continue;
        }
        if (print_rom_log) {
            std::snprintf(line, sizeof line, "load:0x%08x,len:0x%x\r\n", s.addr, s.len);
            soc_.console_write(line);
        }
        buf.resize(s.len);
        fl.read(s.flash_offset, buf.data(), s.len);
        for (uint32_t i = 0; i < s.len; ++i) soc_.write<1>(s.addr + i, buf[i]);
    }
    if (print_rom_log) {
        std::snprintf(line, sizeof line, "entry 0x%08x\r\n", img.entry);
        soc_.console_write(line);
    }
    soc_.start_cpu(img.entry);
}

// ------------------------------------------------------------------ ROM routines

bool Esp32c3Rom::call(Rv32Cpu& cpu) {
    (void)cpu;
    return false;  // no ROM routines are emulated yet
}

// ------------------------------------------------------------------ download mode

void Esp32c3Rom::feed(const uint8_t* data, size_t len) {
    if (!download_mode_) return;
    for (size_t i = 0; i < len; ++i) {
        uint8_t b = data[i];
        if (b == 0xC0) {
            if (in_frame_ && !frame_.empty()) handle_packet(frame_);
            in_frame_ = true;
            escape_ = false;
            frame_.clear();
            if (!download_mode_) return;  // command handed control to user code
            continue;
        }
        if (!in_frame_) continue;
        if (escape_) {
            frame_.push_back(b == 0xDC ? 0xC0 : b == 0xDD ? 0xDB : b);
            escape_ = false;
        } else if (b == 0xDB) {
            escape_ = true;
        } else {
            frame_.push_back(b);
        }
        if (frame_.size() > 0x10000) {  // runaway frame
            in_frame_ = false;
            frame_.clear();
        }
    }
}

void Esp32c3Rom::respond(uint8_t op, uint32_t val, const std::vector<uint8_t>& data, uint8_t status, uint8_t error) {
    std::vector<uint8_t> p;
    uint16_t len = uint16_t(data.size() + 4);
    p.push_back(0x01);
    p.push_back(op);
    p.push_back(uint8_t(len));
    p.push_back(uint8_t(len >> 8));
    put32(p, val);
    p.insert(p.end(), data.begin(), data.end());
    p.push_back(status);
    p.push_back(error);
    p.push_back(0);
    p.push_back(0);
    std::vector<uint8_t> out;
    out.push_back(0xC0);
    for (uint8_t b : p) {
        if (b == 0xC0) { out.push_back(0xDB); out.push_back(0xDC); }
        else if (b == 0xDB) { out.push_back(0xDB); out.push_back(0xDD); }
        else out.push_back(b);
    }
    out.push_back(0xC0);
    soc_.console_write(out.data(), out.size());
}

void Esp32c3Rom::finish_deflate(uint8_t* status, uint8_t* error) {
    if (!defl_active_) return;
    defl_active_ = false;
    std::vector<uint8_t> out;
    if (!zlib_inflate(defl_buf_.data(), defl_buf_.size(), out)) {
        LOGW("rom: FLASH_DEFL data failed to decompress (%zu bytes)", defl_buf_.size());
        *status = 1;
        *error = ERR_DEFLATE;
    } else {
        soc_.flash().program(flash_offset_, out.data(), uint32_t(out.size()));
        LOGI("rom: wrote %zu bytes (%zu compressed) at 0x%x", out.size(), defl_buf_.size(), flash_offset_);
    }
    defl_buf_.clear();
}

void Esp32c3Rom::handle_packet(const std::vector<uint8_t>& p) {
    if (p.size() < 8 || p[0] != 0x00) return;  // not a request
    const uint8_t op = p[1];
    const uint16_t len = uint16_t(p[2] | (p[3] << 8));
    const uint32_t chk = le32(&p[4]);
    if (p.size() < 8u + len) {
        respond(op, 0, {}, 1, ERR_INVALID_MSG);
        return;
    }
    const uint8_t* d = p.data() + 8;
    std::vector<uint8_t> resp;
    uint8_t status = 0, error = 0;
    uint32_t val = 0;
    SpiFlash& fl = soc_.flash();

    LOGT("rom: cmd 0x%02x len %u", op, len);

    // Any command other than more compressed data completes a deflate stream.
    if (defl_active_ && op != FLASH_DEFL_DATA) finish_deflate(&status, &error);

    switch (op) {
    case SYNC:
        if (len < 36 || le32(d) != 0x20120707) return;  // ROM ignores malformed sync
        for (int i = 0; i < 8; ++i) respond(op, 0x20120707, {});
        return;

    case READ_REG:
        if (len < 4) { status = 1; error = ERR_INVALID_MSG; break; }
        val = soc_.read<4>(le32(d));
        break;

    case WRITE_REG:
        for (uint32_t i = 0; i + 16 <= len; i += 16) {
            uint32_t addr = le32(d + i), v = le32(d + i + 4), mask = le32(d + i + 8);
            uint32_t old = mask == 0xFFFFFFFFu ? 0 : soc_.read<4>(addr);
            soc_.write<4>(addr, (old & ~mask) | (v & mask));
        }
        break;

    case MEM_BEGIN:
        if (len < 16) { status = 1; error = ERR_INVALID_MSG; break; }
        mem_size_ = le32(d);
        mem_block_size_ = le32(d + 8);
        mem_offset_ = le32(d + 12);
        LOGI("rom: MEM_BEGIN 0x%08x size 0x%x", mem_offset_, mem_size_);
        break;

    case MEM_DATA: {
        if (len < 16) { status = 1; error = ERR_INVALID_MSG; break; }
        uint32_t n = le32(d), seq = le32(d + 4);
        if (n > len - 16u) { status = 1; error = ERR_INVALID_MSG; break; }
        if (checksum(d + 16, n) != uint8_t(chk)) { status = 1; error = ERR_BAD_CRC; break; }
        uint32_t addr = mem_offset_ + seq * mem_block_size_;
        for (uint32_t i = 0; i < n; ++i) soc_.write<1>(addr + i, d[16 + i]);
        break;
    }

    case MEM_END: {
        uint32_t stay = len >= 4 ? le32(d) : 1;
        uint32_t entry = len >= 8 ? le32(d + 4) : 0;
        respond(op, 0, {});
        if (!stay) {
            LOGI("rom: MEM_END, jumping to 0x%08x", entry);
            download_mode_ = false;
            soc_.start_cpu(entry);
        }
        return;
    }

    case SPI_ATTACH:
    case SPI_SET_PARAMS:
    case CHANGE_BAUDRATE:
        break;

    case FLASH_BEGIN:
    case FLASH_DEFL_BEGIN: {
        if (len < 16) { status = 1; error = ERR_INVALID_MSG; break; }
        flash_erase_size_ = le32(d);
        flash_blocks_ = le32(d + 4);
        flash_block_size_ = le32(d + 8);
        flash_offset_ = le32(d + 12);
        if (flash_offset_ > fl.size() || flash_erase_size_ > fl.size() - flash_offset_) {
            status = 1;
            error = ERR_FLASH_WRITE;
            break;
        }
        // The ROM erases the whole region up front, in 4 KiB sectors.
        uint32_t start = flash_offset_ & ~0xFFFu;
        uint32_t end = (flash_offset_ + flash_erase_size_ + 0xFFF) & ~0xFFFu;
        if (flash_erase_size_) fl.erase(start, end - start);
        if (op == FLASH_DEFL_BEGIN) {
            defl_active_ = true;
            defl_blocks_expected_ = flash_blocks_;
            defl_blocks_got_ = 0;
            defl_buf_.clear();
        }
        LOGI("rom: %s 0x%x, erase 0x%x, %u blocks", op == FLASH_BEGIN ? "FLASH_BEGIN" : "FLASH_DEFL_BEGIN",
             flash_offset_, flash_erase_size_, flash_blocks_);
        break;
    }

    case FLASH_DATA: {
        if (len < 16) { status = 1; error = ERR_INVALID_MSG; break; }
        uint32_t n = le32(d), seq = le32(d + 4);
        if (n > len - 16u) { status = 1; error = ERR_INVALID_MSG; break; }
        if (checksum(d + 16, n) != uint8_t(chk)) { status = 1; error = ERR_BAD_CRC; break; }
        uint32_t addr = flash_offset_ + seq * flash_block_size_;
        if (addr + n > fl.size()) { status = 1; error = ERR_FLASH_WRITE; break; }
        fl.program(addr, d + 16, n);
        break;
    }

    case FLASH_DEFL_DATA: {
        if (len < 16) { status = 1; error = ERR_INVALID_MSG; break; }
        uint32_t n = le32(d);
        if (n > len - 16u) { status = 1; error = ERR_INVALID_MSG; break; }
        if (checksum(d + 16, n) != uint8_t(chk)) { status = 1; error = ERR_BAD_CRC; break; }
        if (!defl_active_) { status = 1; error = ERR_FAILED; break; }
        defl_buf_.insert(defl_buf_.end(), d + 16, d + 16 + n);
        if (++defl_blocks_got_ >= defl_blocks_expected_) finish_deflate(&status, &error);
        break;
    }

    case FLASH_END:
    case FLASH_DEFL_END: {
        uint32_t run_user_code = len >= 4 ? le32(d) : 0;
        respond(op, 0, {}, status, error);
        fl.save();
        download_mode_ = false;
        if (run_user_code) {
            LOGI("rom: leaving download mode, running user code");
            soc_.console_write("\r\n");
            boot_flash();
        } else {
            LOGI("rom: leaving download mode, rebooting");
            soc_.request_reset(BootMode::Flash);
        }
        return;
    }

    case SPI_FLASH_MD5: {
        if (len < 8) { status = 1; error = ERR_INVALID_MSG; break; }
        uint32_t addr = le32(d), size = le32(d + 4);
        if (addr > fl.size() || size > fl.size() - addr) { status = 1; error = ERR_FLASH_READ; break; }
        std::string hex = Md5::hex(fl.data() + addr, size);
        resp.assign(hex.begin(), hex.end());
        break;
    }

    case READ_FLASH_SLOW: {
        if (len < 8) { status = 1; error = ERR_INVALID_MSG; break; }
        uint32_t addr = le32(d), n = le32(d + 4);
        if (n > 64) { status = 1; error = ERR_FLASH_READ; break; }
        resp.assign(64, 0xFF);
        fl.read(addr, resp.data(), n);
        break;
    }

    case GET_SECURITY_INFO:
        put32(resp, 0);                           // flags: no secure boot, no secure download
        resp.push_back(0);                        // flash_crypt_cnt
        for (int i = 0; i < 7; ++i) resp.push_back(0);  // key purposes
        put32(resp, kImageChipId);                // chip id
        put32(resp, 0);                           // api / eco version
        break;

    default:
        LOGW("rom: unsupported loader command 0x%02x", op);
        status = 1;
        error = ERR_INVALID_MSG;
        break;
    }
    respond(op, val, resp, status, error);
}

}  // namespace esp32sim
