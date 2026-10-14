#pragma once
// ESP32-C3 interrupt matrix and CPU interrupt controller (0x600C_2000).
//
// 62 peripheral sources are routed to CPU interrupt lines 1..31 through the
// per-source map registers. Each line has an enable bit, a level/edge type,
// a priority 1..15 and is masked unless its priority is above the threshold.
#include <array>
#include <cstdint>
#include <functional>

#include "soc/peripheral.hpp"

namespace esp32sim::c3 {

namespace irq {
constexpr int GPIO = 16, UART0 = 21, UART1 = 22, USB_SERIAL_JTAG = 26, RTC_CORE = 27;
constexpr int TG0_T0 = 32, TG0_WDT = 33, TG1_T0 = 34, TG1_WDT = 35;
constexpr int SYSTIMER_TARGET0 = 37, SYSTIMER_TARGET1 = 38, SYSTIMER_TARGET2 = 39;
constexpr int DMA_CH0 = 44, SHA = 49, FROM_CPU0 = 50;
constexpr int kSources = 62;
}  // namespace irq

class IntMatrix : public Peripheral {
public:
    explicit IntMatrix(std::string name) : Peripheral(std::move(name)) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;

    void set_source(int source, bool level);
    // Lines that should interrupt the CPU now (highest pending priority).
    std::function<void(uint32_t lines)> on_lines;

private:
    void update();
    std::array<uint8_t, irq::kSources> map_{};
    uint64_t level_ = 0;      // current source levels
    uint32_t edge_latch_ = 0; // edge-type lines latched until CPU_INT_CLEAR
    uint32_t enable_ = 0, type_ = 0, thresh_ = 0, clock_gate_ = 1;
    std::array<uint8_t, 32> pri_{};
    uint32_t raw_lines_ = 0;  // lines with a pending request (EIP status)
};

}  // namespace esp32sim::c3
