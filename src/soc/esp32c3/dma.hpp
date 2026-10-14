#pragma once
// GDMA, the general-purpose DMA controller of the RISC-V ESP32 chips: three
// RX/TX channel pairs that move data between memory and a peripheral (SPI2,
// AES, SHA, ADC, ...) through linked lists of descriptors. The channel
// registers are the same on the C3 and C6; the interrupt registers are one
// block per channel pair on the C3 and separate RX and TX blocks on the C6.
//
// Transfers happen when the peripheral asks: it pulls the bytes software
// queued on the TX ("out") link with out_read() and pushes its results into
// the RX ("in") link with in_write(). Descriptors are updated in memory the
// way the hardware does (length, EOF, owner bit back to the CPU).
#include <cstdint>
#include <functional>

#include "core/bus.hpp"
#include "soc/peripheral.hpp"

namespace esp32sim::c3 {

class Gdma : public Peripheral {
public:
    enum class Layout { C3, C6 };
    static constexpr int kChannels = 3;
    // Peripheral IDs written to IN/OUT_PERI_SEL (SOC_GDMA_TRIG_PERIPH_*).
    enum Periph { SPI2 = 0, UHCI0 = 2, I2S0 = 3, AES = 6, SHA = 7, ADC = 8 };

    // `link_base` supplies the address bits above the 20-bit link address
    // fields (internal SRAM: 0x3FC0_0000 on the C3, 0x4080_0000 on the C6).
    Gdma(std::string name, Bus& bus, uint32_t link_base, Layout layout)
        : Peripheral(std::move(name)), bus_(bus), link_base_(link_base), layout_(layout) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;

    // Copies up to `max` bytes from the TX link of the channel bound to
    // `periph`. Stops early after a descriptor marked EOF. Returns the byte
    // count; *eof is set when the end of the list was reached.
    size_t out_read(int periph, uint8_t* dst, size_t max, bool* eof = nullptr);
    // Stores bytes into the RX link of the channel bound to `periph`; `eof`
    // closes the current descriptor as the end of the frame.
    size_t in_write(int periph, const uint8_t* src, size_t len, bool eof);
    bool has_out(int periph) const { return find(periph, true) >= 0; }

    // Interrupt outputs: C3 index = channel; C6 index = channel (RX) or
    // 3 + channel (TX).
    std::function<void(int index, bool level)> irq;

private:
    struct Dir {
        uint32_t conf0 = 0, conf1 = 0, link = 0, peri_sel = 0x3F;
        uint32_t eof_des = 0, eof_bfr_des = 0;
        uint32_t cur = 0, offset = 0;  // descriptor in use, bytes done in it
        bool active = false;
    };
    struct Channel {
        Dir in, out;
        uint32_t in_raw = 0, in_ena = 0, out_raw = 0, out_ena = 0;  // C6 layout; C3 keeps all in in_raw/in_ena
    };
    int find(int periph, bool out) const;
    void raise(int ch, bool out, uint32_t bit);
    void update_irq(int ch);
    void start(Dir& d);

    Bus& bus_;
    uint32_t link_base_;
    Layout layout_;
    Channel ch_[kChannels];
    uint32_t misc_ = 0;
};

}  // namespace esp32sim::c3
