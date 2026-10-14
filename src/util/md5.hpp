#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace esp32sim {

// RFC 1321 MD5, used by the SPI_FLASH_MD5 loader command.
class Md5 {
public:
    Md5();
    void update(const uint8_t* data, size_t len);
    void final(uint8_t out[16]);
    static std::string hex(const uint8_t* data, size_t len);

private:
    void block(const uint8_t* p);
    uint32_t h_[4];
    uint64_t total_ = 0;
    uint8_t buf_[64];
    size_t buf_len_ = 0;
};

}  // namespace esp32sim
