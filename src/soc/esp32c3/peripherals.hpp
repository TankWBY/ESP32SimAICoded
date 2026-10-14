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
    int loopback_bit = 14;  // CONF0.LOOPBACK (C6: 12)

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

// USB-Serial-JTAG, CDC-ACM side, with a host that is always attached and
// reads every IN packet at once: SOF frames count up once per millisecond,
// a flushed packet empties the IN endpoint (SERIAL_IN_EMPTY), and bytes from
// the host arrive in 64-byte OUT packets (SERIAL_OUT_RECV_PKT).
class UsbSerialJtag : public RegisterFile {
public:
    static constexpr size_t kPacket = 64;
    explicit UsbSerialJtag(std::string name) : RegisterFile(std::move(name)) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    // Host side: the host sends the next OUT packet (up to 64 bytes) once
    // software has emptied the endpoint, at most one per USB frame.
    size_t rx_space() const;
    void push_rx(uint8_t b);
    std::function<void(uint8_t)> on_tx;
    std::function<void(bool level)> irq;
    std::function<uint32_t()> frame_number;  // USB SOF counter (1 kHz)

private:
    void update_irq();
    std::deque<uint8_t> rx_;
    uint32_t int_raw_ = 0;
    uint32_t rx_frame_ = ~0u;  // frame of the last OUT packet
};

// LED PWM controller (LEDC): register state plus what software can observe
// of it. Duty and timer updates take effect at once (DUTY_R follows DUTY),
// and a duty change or fade ends immediately with its DUTY_CHNG_END
// interrupt. C3-style fades (step fields in CONF1) land on their final duty;
// the C6's gamma-RAM fades are not modelled and stay at the start duty.
class Ledc : public RegisterFile {
public:
    static constexpr int kChannels = 6;
    Ledc(std::string name, bool conf1_fade) : RegisterFile(std::move(name)), conf1_fade_(conf1_fade) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    std::function<void(bool level)> irq;

private:
    void update_irq();
    bool conf1_fade_;
    uint32_t int_raw_ = 0;
};

// General-purpose SPI controller (GP-SPI2) in master mode: a transaction
// runs as soon as CMD.USR is set. Data comes from W0-W15 or the GDMA TX link
// and what is clocked in on MISO lands in W0-W15 or the GDMA RX link.
// `exchange` gives the MISO byte for each MOSI byte.
class GpSpi : public RegisterFile {
public:
    explicit GpSpi(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    std::function<uint8_t(uint8_t mosi)> exchange;
    std::function<size_t(uint8_t* dst, size_t len)> dma_read;
    std::function<void(const uint8_t* src, size_t len)> dma_write;
    std::function<void(bool level)> irq;

private:
    void transfer();
    void update_irq();
    uint32_t int_raw_ = 0;
};

// SAR ADC digital controller (APB_SARADC), same layout on the C3 and C6:
// one-shot conversions only. A conversion completes as soon as it is
// started; `sample` supplies the 12-bit result for (unit, channel), and the
// internal calibration channel reads 0.
class SarAdc : public RegisterFile {
public:
    explicit SarAdc(std::string name) : RegisterFile(std::move(name)) {}
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;
    std::function<uint32_t(int unit, int channel)> sample;

private:
    uint32_t int_raw_ = 0;
};

}  // namespace esp32sim::c3
