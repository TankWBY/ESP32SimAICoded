#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace esp32sim {

// Decompress a zlib stream (RFC 1950 wrapper around RFC 1951 DEFLATE), as sent
// by esptool with FLASH_DEFL_* commands. Returns false on malformed input.
bool zlib_inflate(const uint8_t* in, size_t in_len, std::vector<uint8_t>& out);

// Raw DEFLATE without the zlib header.
bool raw_inflate(const uint8_t* in, size_t in_len, std::vector<uint8_t>& out, size_t* consumed = nullptr);

}  // namespace esp32sim
