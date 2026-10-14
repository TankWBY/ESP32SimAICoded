/* Bare-metal ESP32-C3 demo: talks to the hardware registers directly (no
 * ESP-IDF, no ROM calls), so it runs on real silicon and on esp32sim alike.
 *
 *  - prints a banner with the MAC address read from eFuse
 *  - blinks GPIO8 (the RGB/LED pin on most C3 dev boards) every 500 ms
 *    using the 16 MHz SYSTIMER
 *  - echoes what you type; "reset" + Enter triggers a software reset
 */
#include <stdint.h>
#include "regs.h"

#define LED_GPIO 8

static void putc_(char c) {
    while (((UART0_STATUS >> 16) & 0x3FF) >= 126) {}
    UART0_FIFO = (uint8_t)c;
}
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void puthex(uint32_t v, int digits) {
    static const char hex[] = "0123456789abcdef";
    for (int i = digits - 1; i >= 0; --i) putc_(hex[(v >> (4 * i)) & 15]);
}
static void putdec(uint32_t v) {
    char buf[11];
    int n = 0;
    do { buf[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) putc_(buf[--n]);
}

static uint64_t systimer_ticks(void) {   /* 16 MHz */
    SYSTIMER_UNIT0_OP = 1u << 30;        /* latch the counter */
    while (!(SYSTIMER_UNIT0_OP & (1u << 29))) {}
    return ((uint64_t)SYSTIMER_UNIT0_HI << 32) | SYSTIMER_UNIT0_LO;
}

static int uart_getc(void) {
    if ((UART0_STATUS & 0x3FF) == 0) return -1;
    return (int)(UART0_FIFO & 0xFF);
}

static int streq(const char *a, const char *b) {
    while (*a && *a == *b) { ++a; ++b; }
    return *a == *b;
}

const char *const banner = "\r\n*** Hello from a bare-metal ESP32-C3 program! ***\r\n";

int main(void) {
    puts_(banner);
    uint32_t mac0 = EFUSE_MAC0, mac1 = EFUSE_MAC1;
    puts_("MAC address: ");
    uint8_t mac[6] = {(uint8_t)(mac1 >> 8), (uint8_t)mac1, (uint8_t)(mac0 >> 24),
                      (uint8_t)(mac0 >> 16), (uint8_t)(mac0 >> 8), (uint8_t)mac0};
    for (int i = 0; i < 6; ++i) { puthex(mac[i], 2); if (i < 5) putc_(':'); }
    puts_("\r\nReset reason: ");
    putdec(RTC_RESET_STATE & 0x3F);
    puts_("   strap: 0x");
    puthex(GPIO_STRAP, 2);
    puts_("\r\nRunning from 0x");
    puthex((uint32_t)(uintptr_t)&main, 8);
    puts_("\r\nType something (\"reset\" restarts the chip):\r\n> ");

    GPIO_ENABLE_W1TS = 1u << LED_GPIO;
    uint64_t next = systimer_ticks();
    uint32_t seconds = 0;
    int led = 0;
    char line[64];
    unsigned len = 0;

    for (;;) {
        uint64_t now = systimer_ticks();
        if ((int64_t)(now - next) >= 0) {
            next += 8000000;   /* 500 ms */
            led ^= 1;
            if (led) GPIO_OUT_W1TS = 1u << LED_GPIO; else GPIO_OUT_W1TC = 1u << LED_GPIO;
            if (!led && ++seconds % 10 == 0) {
                puts_("\r\n[uptime ");
                putdec(seconds);
                puts_(" s]\r\n> ");
                for (unsigned i = 0; i < len; ++i) putc_(line[i]);
            }
        }
        int c = uart_getc();
        if (c < 0) continue;
        if (c == '\r' || c == '\n') {
            line[len] = 0;
            puts_("\r\n");
            if (streq(line, "reset")) {
                puts_("Resetting...\r\n");
                RTC_CNTL_OPTIONS0 = 1u << 31;   /* RTC_CNTL_SW_SYS_RST */
            } else if (len) {
                puts_("You typed: ");
                puts_(line);
                puts_("\r\n");
            }
            len = 0;
            puts_("> ");
        } else if ((c == 8 || c == 127) && len) {
            --len;
            puts_("\b \b");
        } else if (c >= 32 && len < sizeof line - 1) {
            line[len++] = (char)c;
            putc_((char)c);
        }
    }
}
