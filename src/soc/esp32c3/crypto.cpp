#include "soc/esp32c3/crypto.hpp"

#include <algorithm>
#include <vector>

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

// ---------------------------------------------------------------- AES

namespace {
constexpr uint8_t kSbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9,
    0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f,
    0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07,
    0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3,
    0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58,
    0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3,
    0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f,
    0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88,
    0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac,
    0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a,
    0xae, 0x08, 0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70,
    0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11,
    0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, 0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42,
    0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};

struct InvSbox {
    uint8_t t[256];
    InvSbox() {
        for (int i = 0; i < 256; ++i) t[kSbox[i]] = uint8_t(i);
    }
};
const InvSbox kInvSbox;

inline uint8_t xtime(uint8_t x) { return uint8_t((x << 1) ^ ((x & 0x80) ? 0x1B : 0)); }
inline uint8_t gmul(uint8_t a, uint8_t b) {
    uint8_t r = 0;
    while (b) {
        if (b & 1) r ^= a;
        a = xtime(a);
        b >>= 1;
    }
    return r;
}

void words_to_bytes(const uint32_t* w, int n, uint8_t* out) {
    for (int i = 0; i < n; ++i)
        for (int b = 0; b < 4; ++b) out[4 * i + b] = uint8_t(w[i] >> (8 * b));
}
void bytes_to_words(const uint8_t* in, int n, uint32_t* w) {
    for (int i = 0; i < n; ++i)
        w[i] = uint32_t(in[4 * i]) | uint32_t(in[4 * i + 1]) << 8 | uint32_t(in[4 * i + 2]) << 16 |
               uint32_t(in[4 * i + 3]) << 24;
}

constexpr uint32_t AES_MODE = 0x40, AES_ENDIAN = 0x44, AES_TRIGGER = 0x48, AES_STATE = 0x4C, AES_IV = 0x50,
                   AES_DMA_ENABLE = 0x90, AES_BLOCK_MODE = 0x94, AES_BLOCK_NUM = 0x98, AES_INC_SEL = 0x9C,
                   AES_INT_CLEAR = 0xAC, AES_INT_ENA = 0xB0, AES_DATE = 0xB4, AES_DMA_EXIT = 0xB8;
enum { STATE_IDLE = 0, STATE_DONE = 2 };
enum { ECB, CBC, OFB, CTR, CFB8, CFB128 };
}  // namespace

void Aes::reset() {
    key_.fill(0);
    text_in_.fill(0);
    text_out_.fill(0);
    iv_.fill(0);
    mode_ = state_ = dma_enable_ = block_mode_ = block_num_ = inc_sel_ = int_ena_ = 0;
    int_raw_ = false;
    if (irq) irq(false);
}

uint32_t Aes::read(uint32_t off) {
    if (off < 0x20) return key_[off / 4];
    if (off < 0x30) return text_in_[(off - 0x20) / 4];
    if (off < 0x40) return text_out_[(off - 0x30) / 4];
    if (off >= AES_IV && off < AES_IV + 16) return iv_[(off - AES_IV) / 4];
    switch (off) {
    case AES_MODE: return mode_;
    case AES_STATE: return state_;
    case AES_DMA_ENABLE: return dma_enable_;
    case AES_BLOCK_MODE: return block_mode_;
    case AES_BLOCK_NUM: return block_num_;
    case AES_INC_SEL: return inc_sel_;
    case AES_INT_ENA: return int_ena_;
    case AES_DATE: return 0x20191210;
    default: return 0;
    }
}

void Aes::write(uint32_t off, uint32_t value) {
    if (off < 0x20) { key_[off / 4] = value; return; }
    if (off < 0x30) { text_in_[(off - 0x20) / 4] = value; return; }
    if (off < 0x40) return;
    if (off >= AES_IV && off < AES_IV + 16) { iv_[(off - AES_IV) / 4] = value; return; }
    switch (off) {
    case AES_MODE: mode_ = value & 7; return;
    case AES_TRIGGER:
        if (value & 1) dma_enable_ ? run_dma() : run_typical();
        return;
    case AES_DMA_ENABLE: dma_enable_ = value & 1; return;
    case AES_BLOCK_MODE: block_mode_ = value & 7; return;
    case AES_BLOCK_NUM: block_num_ = value; return;
    case AES_INC_SEL: inc_sel_ = value & 1; return;
    case AES_INT_ENA:
        int_ena_ = value & 1;
        if (irq) irq(int_raw_ && int_ena_);
        return;
    case AES_INT_CLEAR:
        int_raw_ = false;
        if (irq) irq(false);
        return;
    case AES_DMA_EXIT: state_ = STATE_IDLE; return;
    default: return;  // ENDIAN and the GCM registers: not modelled
    }
}

void Aes::expand_key() {
    // MODE bit 1 selects AES-256, bit 2 decryption; AES-192 (bit 0) is not on these chips.
    const int nk = (mode_ & 2) ? 8 : 4;
    rounds_ = nk + 6;
    uint8_t* w = round_keys_.data();
    words_to_bytes(key_.data(), nk, w);
    uint8_t rcon = 1;
    for (int i = nk; i < 4 * (rounds_ + 1); ++i) {
        uint8_t t[4] = {w[4 * (i - 1)], w[4 * (i - 1) + 1], w[4 * (i - 1) + 2], w[4 * (i - 1) + 3]};
        if (i % nk == 0) {
            uint8_t t0 = t[0];
            t[0] = uint8_t(kSbox[t[1]] ^ rcon), t[1] = kSbox[t[2]], t[2] = kSbox[t[3]], t[3] = kSbox[t0];
            rcon = xtime(rcon);
        } else if (nk > 6 && i % nk == 4) {
            for (auto& b : t) b = kSbox[b];
        }
        for (int b = 0; b < 4; ++b) w[4 * i + b] = w[4 * (i - nk) + b] ^ t[b];
    }
}

void Aes::encrypt_block(const uint8_t in[16], uint8_t out[16]) const {
    uint8_t s[16];
    for (int i = 0; i < 16; ++i) s[i] = in[i] ^ round_keys_[size_t(i)];
    for (int r = 1; r <= rounds_; ++r) {
        uint8_t t[16];
        for (int c = 0; c < 4; ++c)  // SubBytes + ShiftRows
            for (int row = 0; row < 4; ++row) t[4 * c + row] = kSbox[s[4 * ((c + row) % 4) + row]];
        if (r != rounds_) {
            for (int c = 0; c < 4; ++c) {  // MixColumns
                uint8_t* col = t + 4 * c;
                uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3], x = a0 ^ a1 ^ a2 ^ a3;
                col[0] ^= x ^ xtime(a0 ^ a1), col[1] ^= x ^ xtime(a1 ^ a2);
                col[2] ^= x ^ xtime(a2 ^ a3), col[3] ^= x ^ xtime(a3 ^ a0);
            }
        }
        for (int i = 0; i < 16; ++i) s[i] = t[i] ^ round_keys_[size_t(16 * r + i)];
    }
    std::copy(s, s + 16, out);
}

void Aes::decrypt_block(const uint8_t in[16], uint8_t out[16]) const {
    uint8_t s[16];
    for (int i = 0; i < 16; ++i) s[i] = in[i] ^ round_keys_[size_t(16 * rounds_ + i)];
    for (int r = rounds_ - 1; r >= 0; --r) {
        uint8_t t[16];
        for (int c = 0; c < 4; ++c)  // InvShiftRows + InvSubBytes
            for (int row = 0; row < 4; ++row) t[4 * ((c + row) % 4) + row] = kInvSbox.t[s[4 * c + row]];
        for (int i = 0; i < 16; ++i) t[i] ^= round_keys_[size_t(16 * r + i)];
        if (r != 0) {
            for (int c = 0; c < 4; ++c) {  // InvMixColumns
                uint8_t* col = t + 4 * c;
                uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
                col[0] = gmul(a0, 14) ^ gmul(a1, 11) ^ gmul(a2, 13) ^ gmul(a3, 9);
                col[1] = gmul(a0, 9) ^ gmul(a1, 14) ^ gmul(a2, 11) ^ gmul(a3, 13);
                col[2] = gmul(a0, 13) ^ gmul(a1, 9) ^ gmul(a2, 14) ^ gmul(a3, 11);
                col[3] = gmul(a0, 11) ^ gmul(a1, 13) ^ gmul(a2, 9) ^ gmul(a3, 14);
            }
        }
        std::copy(t, t + 16, s);
    }
    std::copy(s, s + 16, out);
}

void Aes::run_typical() {
    expand_key();
    uint8_t in[16], out[16];
    words_to_bytes(text_in_.data(), 4, in);
    (mode_ & 4) ? decrypt_block(in, out) : encrypt_block(in, out);
    bytes_to_words(out, 4, text_out_.data());
}

void Aes::run_dma() {
    expand_key();
    const bool dec = (mode_ & 4) != 0;
    const size_t len = size_t(block_num_) * 16;
    std::vector<uint8_t> data(len), out(len);
    size_t got = dma_read ? dma_read(data.data(), len) : 0;
    if (got < len) LOGW("aes: DMA input short: %zu of %zu bytes", got, len);
    uint8_t iv[16];
    words_to_bytes(iv_.data(), 4, iv);
    for (size_t off = 0; off < len; off += 16) {
        const uint8_t* in = data.data() + off;
        uint8_t* o = out.data() + off;
        uint8_t ks[16];
        switch (block_mode_) {
        case ECB: dec ? decrypt_block(in, o) : encrypt_block(in, o); break;
        case CBC:
            if (dec) {
                decrypt_block(in, o);
                for (int i = 0; i < 16; ++i) o[i] ^= iv[i];
                std::copy(in, in + 16, iv);
            } else {
                uint8_t x[16];
                for (int i = 0; i < 16; ++i) x[i] = in[i] ^ iv[i];
                encrypt_block(x, o);
                std::copy(o, o + 16, iv);
            }
            break;
        case OFB:
            encrypt_block(iv, iv);
            for (int i = 0; i < 16; ++i) o[i] = in[i] ^ iv[i];
            break;
        case CTR: {
            encrypt_block(iv, ks);
            for (int i = 0; i < 16; ++i) o[i] = in[i] ^ ks[i];
            // INC_SEL 0 counts in the low 32 bits, 1 in all 128 (big-endian).
            for (int i = 15; i >= (inc_sel_ ? 0 : 12); --i)
                if (++iv[i]) break;
            break;
        }
        case CFB8:
            for (int b = 0; b < 16; ++b) {
                encrypt_block(iv, ks);
                o[b] = in[b] ^ ks[0];
                std::copy(iv + 1, iv + 16, iv);
                iv[15] = dec ? in[b] : o[b];
            }
            break;
        case CFB128:
            encrypt_block(iv, ks);
            for (int i = 0; i < 16; ++i) o[i] = in[i] ^ ks[i], iv[i] = dec ? in[i] : o[i];
            break;
        default:
            LOGW("aes: block mode %u not modelled", block_mode_);
            break;
        }
    }
    bytes_to_words(iv, 4, iv_.data());
    if (dma_write) dma_write(out.data(), len);
    state_ = STATE_DONE;
    int_raw_ = true;
    if (irq) irq(int_ena_ != 0);
}

}  // namespace esp32sim::c3
