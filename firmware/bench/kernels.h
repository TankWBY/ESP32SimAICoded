#pragma once
#include <stdint.h>

/* One unit of work; the result depends only on `seed`. */
uint32_t bench_iteration(uint32_t seed);
