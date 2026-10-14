/* A handful of ESP32-C3 register addresses (ESP32-C3 Technical Reference Manual). */
#pragma once
#include <stdint.h>

#define REG(a) (*(volatile uint32_t *)(a))

#define UART0_FIFO        REG(0x60000000)
#define UART0_STATUS      REG(0x6000001C)   /* [9:0] rxfifo_cnt, [25:16] txfifo_cnt */

#define GPIO_OUT_W1TS     REG(0x60004008)
#define GPIO_OUT_W1TC     REG(0x6000400C)
#define GPIO_ENABLE_W1TS  REG(0x60004024)
#define GPIO_STRAP        REG(0x60004038)

#define RTC_CNTL_OPTIONS0 REG(0x60008000)
#define RTC_RESET_STATE   REG(0x60008038)
#define EFUSE_MAC0        REG(0x60008844)
#define EFUSE_MAC1        REG(0x60008848)

#define SYSTIMER_UNIT0_OP REG(0x60023004)
#define SYSTIMER_UNIT0_HI REG(0x60023040)
#define SYSTIMER_UNIT0_LO REG(0x60023044)
