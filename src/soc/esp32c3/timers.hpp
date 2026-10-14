#pragma once
// ESP32-C3 timers: SYSTIMER and the two timer groups (TIMG0/1).
//
// Time is the SoC master cycle counter (ticks of the fastest CPU clock, see
// Rv32Cpu::cycles). Timers compute their values from it lazily and report
// when their next alarm is due so the chip can stop the CPU right there.
#include <cstdint>
#include <functional>
#include <limits>

#include "soc/peripheral.hpp"

namespace esp32sim::c3 {

struct Clocks {
    const uint64_t* now = nullptr;  // master cycles
    uint32_t master_hz = 160000000;
    uint32_t apb_hz = 40000000;
    static constexpr uint32_t kXtalHz = 40000000;
};

class TimedDevice {
public:
    static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
    virtual ~TimedDevice() = default;
    virtual uint64_t next_event() const = 0;  // master cycle of the next alarm
    virtual void tick() = 0;                  // handle everything due by now
};

using IrqOut = std::function<void(int source, bool level)>;

class Systimer : public RegisterFile, public TimedDevice {
public:
    Systimer(std::string name, const Clocks& clk) : RegisterFile(std::move(name)), clk_(clk) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    uint64_t next_event() const override;
    void tick() override;
    IrqOut irq;

private:
    // 16 MHz counter, derived from master cycles.
    uint64_t div() const { return clk_.master_hz / 16000000u; }
    uint64_t ticks() const { return *clk_.now / div(); }
    uint64_t unit_value(int unit) const { return (ticks() + offset_[unit]) & kMask; }
    void update_irq();
    static constexpr uint64_t kMask = (1ull << 52) - 1;
    const Clocks& clk_;
    uint64_t offset_[2] = {0, 0};
    uint64_t snapshot_[2] = {0, 0};
    uint64_t target_[3] = {0, 0, 0};  // active comparator values
    uint32_t fired_ = 0;              // one-shot comparators that already fired
    uint32_t int_raw_ = 0, int_ena_ = 0;
};

class Timg : public RegisterFile, public TimedDevice {
public:
    Timg(std::string name, const Clocks& clk, int t0_source, int wdt_source)
        : RegisterFile(std::move(name)), clk_(clk), t0_src_(t0_source), wdt_src_(wdt_source) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    uint64_t next_event() const override;
    void tick() override;
    IrqOut irq;
    // Watchdog reset; `system` is false for a CPU-only reset.
    std::function<void(bool system)> on_wdt_reset;

private:
    // Timer 0
    double t0_rate() const;  // counter ticks per master cycle (0 when stopped)
    uint64_t t0_value() const;
    void t0_settle();        // fold elapsed time into t0_base_
    uint64_t t0_base_ = 0, t0_mark_ = 0;
    // Watchdog
    uint64_t wdt_stage_end() const;
    void wdt_restart();
    int wdt_stage_ = 0;
    uint64_t wdt_start_ = 0;
    void update_irq();
    const Clocks& clk_;
    int t0_src_, wdt_src_;
    uint32_t int_raw_ = 0, int_ena_ = 0;
};

}  // namespace esp32sim::c3
