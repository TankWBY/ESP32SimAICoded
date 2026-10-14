#include "soc/esp32c3/crypto.hpp"

#include "util/log.hpp"

namespace esp32sim::c3 {
namespace {
inline uint32_t bswap(uint32_t v) { return (v >> 24) | ((v >> 8) & 0xFF00) | ((v << 8) & 0xFF0000) | (v << 24); }
inline uint32_t rotr(uint32_t v, int n) { return (v >> n) | (v << (32 - n)); }
inline uint32_t rotl(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }

constexpr uint32_t kSha256K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
    0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
    0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
constexpr uint32_t kSha256Init[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                     0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
constexpr uint32_t kSha224Init[8] = {0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939,
                                     0xffc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4};
constexpr uint32_t kSha1Init[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
}  // namespace

void Sha::reset() {
    mode_ = block_num_ = int_ena_ = 0;
    h_.fill(0);
    m_.fill(0);
}

uint32_t Sha::read(uint32_t off) {
    switch (off) {
    case 0x00: return mode_;
    case 0x0C: return block_num_;
    case 0x18: return 0;  // BUSY: operations complete instantly
    case 0x28: return int_ena_;
    case 0x2C: return 0x20190402;  // DATE
    default: break;
    }
    if (off >= 0x40 && off < 0x60) return h_[(off - 0x40) / 4];
    if (off >= 0x80 && off < 0xC0) return m_[(off - 0x80) / 4];
    return 0;
}

void Sha::write(uint32_t off, uint32_t value) {
    switch (off) {
    case 0x00: mode_ = value & 7; return;
    case 0x0C: block_num_ = value; return;
    case 0x10: process(true); return;   // START
    case 0x14: process(false); return;  // CONTINUE
    case 0x1C:                          // DMA_START
    case 0x20: {                        // DMA_CONTINUE
        bool first = off == 0x1C;
        for (uint32_t i = 0; i < block_num_; ++i) {
            uint8_t block[64];
            if (!dma_read || !dma_read(block)) {
                LOGW("sha: DMA input not available");
                return;
            }
            for (int w = 0; w < 16; ++w)
                m_[size_t(w)] = uint32_t(block[w * 4]) | uint32_t(block[w * 4 + 1]) << 8 |
                                uint32_t(block[w * 4 + 2]) << 16 | uint32_t(block[w * 4 + 3]) << 24;
            process(first && i == 0);
        }
        return;
    }
    case 0x28: int_ena_ = value & 1; return;
    default: break;
    }
    if (off >= 0x40 && off < 0x60) h_[(off - 0x40) / 4] = value;
    else if (off >= 0x80 && off < 0xC0) m_[(off - 0x80) / 4] = value;
}

void Sha::process(bool first) {
    if (first) {
        h_.fill(0);
        if (mode_ == 0) for (int i = 0; i < 5; ++i) h_[size_t(i)] = bswap(kSha1Init[i]);
        else for (int i = 0; i < 8; ++i) h_[size_t(i)] = bswap(mode_ == 1 ? kSha224Init[i] : kSha256Init[i]);
    }
    compress();
}

void Sha::compress() {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) w[i] = bswap(m_[size_t(i)]);
    if (mode_ == 0) {  // SHA-1
        for (int i = 16; i < 80; ++i) w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        uint32_t a = bswap(h_[0]), b = bswap(h_[1]), c = bswap(h_[2]), d = bswap(h_[3]), e = bswap(h_[4]);
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20) f = (b & c) | (~b & d), k = 0x5A827999;
            else if (i < 40) f = b ^ c ^ d, k = 0x6ED9EBA1;
            else if (i < 60) f = (b & c) | (b & d) | (c & d), k = 0x8F1BBCDC;
            else f = b ^ c ^ d, k = 0xCA62C1D6;
            uint32_t t = rotl(a, 5) + f + e + k + w[i];
            e = d, d = c, c = rotl(b, 30), b = a, a = t;
        }
        uint32_t add[5] = {a, b, c, d, e};
        for (int i = 0; i < 5; ++i) h_[size_t(i)] = bswap(bswap(h_[size_t(i)]) + add[i]);
        return;
    }
    for (int i = 16; i < 64; ++i) {
        uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t s[8];
    for (int i = 0; i < 8; ++i) s[i] = bswap(h_[size_t(i)]);
    uint32_t a = s[0], b = s[1], c = s[2], d = s[3], e = s[4], f = s[5], g = s[6], h = s[7];
    for (int i = 0; i < 64; ++i) {
        uint32_t t1 = h + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + kSha256K[i] + w[i];
        uint32_t t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        h = g, g = f, f = e, e = d + t1, d = c, c = b, b = a, a = t1 + t2;
    }
    uint32_t add[8] = {a, b, c, d, e, f, g, h};
    for (int i = 0; i < 8; ++i) h_[size_t(i)] = bswap(s[i] + add[i]);
}

}  // namespace esp32sim::c3
