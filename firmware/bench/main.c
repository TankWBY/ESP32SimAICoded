/* Bare-metal ESP32-C3 CPU benchmark (see README.md). Runs rounds of 100
 * kernel iterations forever and prints, per round, the checksum (which must
 * match the host build) and the rate measured on the 16 MHz SYSTIMER. */
#include <stdint.h>
#include "../hello/regs.h"
#include "kernels.h"

#define EXPECTED 0xdcfd2453u  /* cc -O2 host_check.c kernels.c && ./a.out */

static void putc_(char c) {
    while (((UART0_STATUS >> 16) & 0x3FF) >= 126) {}
    UART0_FIFO = (uint8_t)c;
}
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void puthex(uint32_t v) {
    static const char hex[] = "0123456789abcdef";
    for (int i = 7; i >= 0; --i) putc_(hex[(v >> (4 * i)) & 15]);
}
static void putdec(uint32_t v) {
    char buf[11];
    int n = 0;
    do { buf[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) putc_(buf[--n]);
}

static uint64_t systimer_ticks(void) {   /* 16 MHz */
    SYSTIMER_UNIT0_OP = 1u << 30;
    while (!(SYSTIMER_UNIT0_OP & (1u << 29))) {}
    return ((uint64_t)SYSTIMER_UNIT0_HI << 32) | SYSTIMER_UNIT0_LO;
}

static uint32_t instret(void) {
    uint32_t v;
    __asm__ volatile("csrr %0, minstret" : "=r"(v));
    return v;
}

/* Run the CPU from the PLL at 160 MHz, as ESP-IDF and Arduino apps do (the
 * ROM leaves it on XTAL/2 = 20 MHz). */
static void cpu_160mhz(void) {
    REG(0x600C0008) = (1u << 2) | 1u;  /* SYSTEM_CPU_PER_CONF: PLL 480 MHz, CPU period 160 MHz */
    REG(0x600C0058) = (REG(0x600C0058) & ~0xFFFu) | (1u << 10);  /* SYSTEM_SYSCLK_CONF: SOC_CLK = PLL */
}

int main(void) {
    cpu_160mhz();
    puts_("\r\nbench: CPU benchmark, 100 iterations per round\r\n");
    for (uint32_t round = 1;; ++round) {
        uint64_t t0 = systimer_ticks();
        uint32_t i0 = instret();
        uint32_t sum = 0;
        for (uint32_t i = 0; i < 100; ++i) sum += bench_iteration(i);
        uint32_t insns = instret() - i0;
        uint32_t us = (uint32_t)((systimer_ticks() - t0) / 16);
        puts_("bench: round ");
        putdec(round);
        puts_(sum == EXPECTED ? " ok " : " BAD ");
        puthex(sum);
        puts_(", ");
        putdec(us);
        puts_(" us, ");
        putdec(insns);
        puts_(" instructions\r\n");
    }
}
