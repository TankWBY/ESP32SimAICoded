// SPI NOR flash chip model and the SPI memory controller ("SPI1") that the
// ROM loader, esptool and firmware use to talk to it.
#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "soc/peripheral.hpp"

namespace esp32sim {

class SpiFlash {
public:
    explicit SpiFlash(uint32_t size_bytes = 4 * 1024 * 1024);

    bool load(const std::string& path);  // missing file -> erased flash
    bool save();
    void set_path(const std::string& path) { path_ = path; }

    uint32_t size() const { return uint32_t(data_.size()); }
    uint8_t* data() { return data_.data(); }
    const uint8_t* data() const { return data_.data(); }
    uint32_t jedec_id() const { return jedec_; }

    // Raw operations, NOR semantics (program can only clear bits).
    void erase(uint32_t addr, uint32_t len);
    void program(uint32_t addr, const uint8_t* src, uint32_t len);
    void read(uint32_t addr, uint8_t* dst, uint32_t len) const;
    bool dirty() const { return dirty_; }

    // One SPI transaction: opcode, optional address, MOSI bytes in, MISO bytes out.
    void transaction(uint8_t cmd, uint32_t addr, bool has_addr, const std::vector<uint8_t>& mosi,
                     std::vector<uint8_t>& miso, size_t miso_len);

    uint8_t status() const { return status_; }
    std::function<void()> on_modified;  // contents changed (erase/program)

private:
    std::vector<uint8_t> data_;
    std::string path_;
    uint32_t jedec_;
    uint8_t status_ = 0;  // bit0 WIP, bit1 WEL
    uint8_t status2_ = 0x02;  // QE set
    bool dirty_ = false;
};

// Register layout of an Espressif SPI_MEM controller (C3/S3/C6 style).
struct SpiMemLayout {
    uint32_t cmd = 0x00, addr = 0x04, user = 0x18, user1 = 0x1C, user2 = 0x20;
    uint32_t mosi_dlen = 0x24, miso_dlen = 0x28, rd_status = 0x2C, w0 = 0x58;
};

class SpiMemController : public RegisterFile {
public:
    SpiMemController(std::string name, SpiFlash& flash, SpiMemLayout layout = {});
    uint32_t read(uint32_t offset) override;
    void write(uint32_t offset, uint32_t value) override;

private:
    void execute(uint32_t cmd_bits);
    uint8_t wbyte(size_t i);
    void set_wbyte(size_t i, uint8_t v);

    SpiFlash& flash_;
    SpiMemLayout l_;
};

}  // namespace esp32sim
