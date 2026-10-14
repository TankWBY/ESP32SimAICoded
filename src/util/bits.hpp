#pragma once
#include <cstdint>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace esp32sim {

// Index of the lowest set bit; v must be non-zero.
inline int ctz32(uint32_t v) {
#if defined(_MSC_VER)
    unsigned long i;
    _BitScanForward(&i, v);
    return int(i);
#else
    return __builtin_ctz(v);
#endif
}

inline int ctz64(uint64_t v) {
#if defined(_MSC_VER)
    unsigned long i;
    _BitScanForward64(&i, v);
    return int(i);
#else
    return __builtin_ctzll(v);
#endif
}

}  // namespace esp32sim
