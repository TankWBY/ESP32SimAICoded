// Small canonical-Huffman DEFLATE decoder, modelled on zlib's "puff".
#include "util/inflate.hpp"

namespace esp32sim {

namespace {

struct Huffman {
    uint16_t count[16];
    uint16_t symbol[320];
};

struct State {
    const uint8_t* in;
    size_t in_len;
    size_t pos = 0;
    uint32_t bitbuf = 0;
    int bitcnt = 0;
    bool error = false;
    std::vector<uint8_t>* out;

    int bits(int need) {
        uint32_t val = bitbuf;
        while (bitcnt < need) {
            if (pos >= in_len) { error = true; return 0; }
            val |= uint32_t(in[pos++]) << bitcnt;
            bitcnt += 8;
        }
        bitbuf = val >> need;
        bitcnt -= need;
        return int(val & ((1u << need) - 1));
    }

    int decode(const Huffman& h) {
        int code = 0, first = 0, index = 0;
        for (int len = 1; len < 16; ++len) {
            code |= bits(1);
            if (error) return -1;
            int count = h.count[len];
            if (code - count < first) return h.symbol[index + (code - first)];
            index += count;
            first += count;
            first <<= 1;
            code <<= 1;
        }
        error = true;
        return -1;
    }
};

int construct(Huffman& h, const uint16_t* length, int n) {
    for (int len = 0; len < 16; ++len) h.count[len] = 0;
    for (int s = 0; s < n; ++s) h.count[length[s]]++;
    if (h.count[0] == n) return 0;
    int left = 1;
    for (int len = 1; len < 16; ++len) {
        left <<= 1;
        left -= h.count[len];
        if (left < 0) return left;
    }
    uint16_t offs[16];
    offs[1] = 0;
    for (int len = 1; len < 15; ++len) offs[len + 1] = uint16_t(offs[len] + h.count[len]);
    for (int s = 0; s < n; ++s)
        if (length[s]) h.symbol[offs[length[s]]++] = uint16_t(s);
    return left;
}

const uint16_t kLenBase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
const uint16_t kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
const uint16_t kDistBase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
const uint16_t kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

bool codes(State& s, const Huffman& lencode, const Huffman& distcode) {
    auto& out = *s.out;
    for (;;) {
        int sym = s.decode(lencode);
        if (sym < 0) return false;
        if (sym < 256) {
            out.push_back(uint8_t(sym));
        } else if (sym == 256) {
            return true;
        } else {
            sym -= 257;
            if (sym >= 29) return false;
            int len = kLenBase[sym] + s.bits(kLenExtra[sym]);
            int dsym = s.decode(distcode);
            if (dsym < 0 || dsym >= 30) return false;
            size_t dist = kDistBase[dsym] + size_t(s.bits(kDistExtra[dsym]));
            if (s.error || dist > out.size()) return false;
            size_t from = out.size() - dist;
            for (int i = 0; i < len; ++i) out.push_back(out[from + size_t(i)]);
        }
    }
}

bool stored(State& s) {
    s.bitbuf = 0;
    s.bitcnt = 0;
    if (s.pos + 4 > s.in_len) return false;
    unsigned len = s.in[s.pos] | (s.in[s.pos + 1] << 8);
    unsigned nlen = s.in[s.pos + 2] | (s.in[s.pos + 3] << 8);
    s.pos += 4;
    if (len != (~nlen & 0xFFFF) || s.pos + len > s.in_len) return false;
    s.out->insert(s.out->end(), s.in + s.pos, s.in + s.pos + len);
    s.pos += len;
    return true;
}

bool fixed(State& s) {
    static Huffman lencode, distcode;
    static bool built = false;
    if (!built) {
        uint16_t lengths[320];
        int sym = 0;
        for (; sym < 144; ++sym) lengths[sym] = 8;
        for (; sym < 256; ++sym) lengths[sym] = 9;
        for (; sym < 280; ++sym) lengths[sym] = 7;
        for (; sym < 288; ++sym) lengths[sym] = 8;
        construct(lencode, lengths, 288);
        for (sym = 0; sym < 30; ++sym) lengths[sym] = 5;
        construct(distcode, lengths, 30);
        built = true;
    }
    return codes(s, lencode, distcode);
}

bool dynamic(State& s) {
    static const uint8_t order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    uint16_t lengths[320];
    int nlen = s.bits(5) + 257, ndist = s.bits(5) + 1, ncode = s.bits(4) + 4;
    if (s.error || nlen > 286 || ndist > 30) return false;
    int idx;
    for (idx = 0; idx < ncode; ++idx) lengths[order[idx]] = uint16_t(s.bits(3));
    for (; idx < 19; ++idx) lengths[order[idx]] = 0;
    Huffman lencode, distcode;
    if (construct(lencode, lengths, 19) != 0) return false;
    idx = 0;
    while (idx < nlen + ndist) {
        int sym = s.decode(lencode);
        if (sym < 0) return false;
        if (sym < 16) {
            lengths[idx++] = uint16_t(sym);
        } else {
            uint16_t len = 0;
            int rep;
            if (sym == 16) {
                if (idx == 0) return false;
                len = lengths[idx - 1];
                rep = 3 + s.bits(2);
            } else if (sym == 17) {
                rep = 3 + s.bits(3);
            } else {
                rep = 11 + s.bits(7);
            }
            if (idx + rep > nlen + ndist) return false;
            while (rep--) lengths[idx++] = len;
        }
    }
    if (lengths[256] == 0) return false;
    int err = construct(lencode, lengths, nlen);
    if (err < 0 || (err > 0 && nlen - lencode.count[0] != 1)) return false;
    err = construct(distcode, lengths + nlen, ndist);
    if (err < 0 || (err > 0 && ndist - distcode.count[0] != 1)) return false;
    return codes(s, lencode, distcode);
}

}  // namespace

bool raw_inflate(const uint8_t* in, size_t in_len, std::vector<uint8_t>& out, size_t* consumed) {
    State s;
    s.in = in;
    s.in_len = in_len;
    s.out = &out;
    int last;
    do {
        last = s.bits(1);
        int type = s.bits(2);
        if (s.error) return false;
        bool ok = type == 0 ? stored(s) : type == 1 ? fixed(s) : type == 2 ? dynamic(s) : false;
        if (!ok || s.error) return false;
    } while (!last);
    if (consumed) *consumed = s.pos;
    return true;
}

bool zlib_inflate(const uint8_t* in, size_t in_len, std::vector<uint8_t>& out) {
    if (in_len < 6) return false;
    if ((in[0] & 0x0F) != 8 || ((in[0] << 8) | in[1]) % 31 != 0 || (in[1] & 0x20)) return false;
    size_t used = 0;
    size_t start = out.size();
    if (!raw_inflate(in + 2, in_len - 2, out, &used)) return false;
    size_t p = 2 + used;
    if (p + 4 > in_len) return false;
    uint32_t want = (uint32_t(in[p]) << 24) | (uint32_t(in[p + 1]) << 16) | (uint32_t(in[p + 2]) << 8) | in[p + 3];
    uint32_t a = 1, b = 0;
    for (size_t i = start; i < out.size(); ++i) {
        a = (a + out[i]) % 65521;
        b = (b + a) % 65521;
    }
    return ((b << 16) | a) == want;
}

}  // namespace esp32sim
