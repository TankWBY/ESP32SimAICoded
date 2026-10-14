// ESP32-C3 peripheral models (register layouts from the ESP32-C3 TRM).
#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>

#include "soc/peripheral.hpp"

namespace esp32sim::c3 {

class Uart : public RegisterFile {
public:
    static constexpr size_t kFifoSize = 128;
    explicit Uart(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;

    // Host side.
    size_t rx_space() const { return kFifoSize - rx_.size(); }
    void push_rx(uint8_t b) {
        rx_.push_back(b);
        update_irq();
    }
    std::function<void(uint8_t)> on_tx;
    std::function<void(bool level)> irq;  // INT_ST != 0

private:
    uint32_t int_raw();
    void update_irq();
    std::deque<uint8_t> rx_;
    uint32_t int_latched_ = 0;
};


class Efuse : public RegisterFile {
public:
    // `sys3` is EFUSE_RD_MAC_SPI_SYS_3 (chip revision, package; layout per chip).
    Efuse(std::string name, const std::array<uint8_t, 6>& mac, uint32_t sys3);
    void write(uint32_t off, uint32_t value) override {}  // read-only model
    void reset() override {}
};

class RtcCntl : public RegisterFile {
public:
    explicit RtcCntl(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    std::function<void(double delay_s)> on_system_reset;
    // RTC watchdog stage 0: delay until reset, or a negative value to disarm.
    std::function<void(double delay_s)> on_wdt;
    std::function<uint64_t()> slow_ticks;  // RTC timer: slow clock cycles since power-on
    uint32_t reset_reason = 1;  // POWERON
    static constexpr double kSlowClockHz = 136000.0;  // RC_SLOW, uncalibrated

private:
    void update_wdt();
};

// Flash MMU: 128 entries of 64 KiB pages, shared by the IBUS (0x4200_0000)
// and DBUS (0x3C00_0000) windows.
class Mmu : public Peripheral {
public:
    static constexpr int kEntries = 128;
    static constexpr uint32_t kInvalid = 1u << 8;
    explicit Mmu(std::string name) : Peripheral(std::move(name)) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    void set_entry(int index, uint32_t value);
    std::function<void(int index, uint32_t value)> on_change;

private:
    std::array<uint32_t, kEntries> table_{};
};

// Cache controller (EXTMEM). The cache itself is not modelled (flash reads are
// always coherent), so maintenance operations complete instantly.
class Extmem : public RegisterFile {
public:
    explicit Extmem(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
};

// SYSTEM registers: clock tree (CPU/APB frequency) and the four
// FROM_CPU software interrupt sources.
class SystemReg : public RegisterFile {
public:
    explicit SystemReg(std::string name) : RegisterFile(std::move(name)) {}
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    uint32_t cpu_hz() const;
    uint32_t apb_hz() const;
    std::function<void()> on_clock_change;
    std::function<void(int index, bool level)> on_cpu_intr;
};

// A register block holding the hardware RNG (WDEV_RND): SYSCON/APB_CTRL +0xB0
// on the C3, LPPERI +0x8 on the C6. Everything else is plain storage.
class Syscon : public RegisterFile {
public:
    Syscon(std::string name, uint32_t rng_offset) : RegisterFile(std::move(name)), rng_off_(rng_offset) {}
    uint32_t read(uint32_t off) override;

private:
    uint32_t rng_off_;
    uint64_t rng_state_ = 0x9E3779B97F4A7C15ull;
};

class UsbSerialJtag : public RegisterFile {
public:
    explicit UsbSerialJtag(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    std::function<void(uint8_t)> on_tx;
};

}  // namespace esp32sim::c3
