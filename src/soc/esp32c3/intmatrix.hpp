#pragma once
// Interrupt matrix and CPU interrupt controller.
//
// Peripheral sources (62 on the C3, 77 on the C6) are routed to CPU interrupt
// lines 1..31 through the per-source map registers. Each line has an enable
// bit, a level/edge type, a priority 1..15 and is masked unless its priority
// is at or above the threshold. On the C3 both live in one block
// (0x600C_2000); on the C6 the matrix is at 0x6001_0000 and the line
// controller (PLIC) at 0x2000_1000, which forwards here with the C3 offsets.
#include <array>
#include <cstdint>
#include <functional>

#include "soc/peripheral.hpp"

namespace esp32sim::c3 {

namespace irq {
constexpr int GPIO = 16, UART0 = 21, UART1 = 22, USB_SERIAL_JTAG = 26, RTC_CORE = 27;
constexpr int TG0_T0 = 32, TG0_WDT = 33, TG1_T0 = 34, TG1_WDT = 35;
constexpr int SYSTIMER_TARGET0 = 37, SYSTIMER_TARGET1 = 38, SYSTIMER_TARGET2 = 39;
constexpr int SPI2 = 19, LEDC = 23, I2C_EXT0 = 29, APB_ADC = 43, DMA_CH0 = 44, AES = 48, SHA = 49, FROM_CPU0 = 50;
constexpr int kSources = 62;
}  // namespace irq

class IntMatrix : public Peripheral {
public:
    static constexpr int kMaxSources = 128;
    // `cpu_regs_inline`: the line controller registers follow the map
    // registers in the same block (C3 layout).
    IntMatrix(std::string name, int sources, bool cpu_regs_inline)
        : Peripheral(std::move(name)), sources_(sources), inline_(cpu_regs_inline) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    // Line controller registers, C3 offsets (ENABLE 0x104 ... THRESH 0x194).
    uint32_t cpu_read(uint32_t off);
    void cpu_write(uint32_t off, uint32_t value);

    void set_source(int source, bool level);
    // Lines that should interrupt the CPU now (highest pending priority).
    std::function<void(uint32_t lines)> on_lines;

private:
    void update();
    int sources_;
    bool inline_;
    std::array<uint8_t, kMaxSources> map_{};
    uint64_t level_[2] = {0, 0};  // current source levels
    uint32_t edge_latch_ = 0; // edge-type lines latched until CPU_INT_CLEAR
    uint32_t enable_ = 0, type_ = 0, thresh_ = 0, clock_gate_ = 1;
    std::array<uint8_t, 32> pri_{};
    uint32_t raw_lines_ = 0;  // lines with a pending request (EIP status)
};

}  // namespace esp32sim::c3
