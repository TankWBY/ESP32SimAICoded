#pragma once
// GPIO of the RISC-V ESP32 chips: the GPIO block (output/enable/input
// registers, per-pin interrupt config, the GPIO matrix) and IO_MUX (pad
// function, pulls, input enable). The register layout is shared by the C3
// (22 pads) and the C6 (31 pads); what differs per chip is described by a
// GpioPads table. Pads have a resolved level that combines what the chip
// drives, the pulls, and an external drive set by the UI.
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "soc/chip.hpp"
#include "soc/peripheral.hpp"

namespace esp32sim::c3 {

struct SignalName {
    int index;
    const char* name;
};
// Per-chip pad description.
struct GpioPads {
    int pins;
    const char* const* func0;     // IO_MUX function 0 of each pad ("GPIOn" = the GPIO matrix)
    const char* const* func2;     // function 2 (nullptr where none)
    uint32_t (*default_mux)(int pin);  // IO_MUX_GPIOn reset value
    uint32_t flash_pads;          // pads wired to the SPI flash
    int usb_dm, usb_dp;           // USB-Serial-JTAG pads
    int u0txd;                    // pad whose function 0 is U0TXD (idles high)
    const SignalName* out_signals;
    size_t out_signal_count;
    uint32_t date;                // GPIO_DATE_REG
};
extern const GpioPads kEsp32c3Pads;
extern const SignalName kOutSignals[];
extern const size_t kOutSignalCount;

class Gpio : public Peripheral {
public:
    static constexpr int kMaxPins = 32;
    Gpio(std::string name, const GpioPads& pads);
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;

    Peripheral* iomux() { return &iomux_; }
    void set_strap(uint32_t strap) { strap_ = strap; }
    void set_external(int pin, int level);  // -1 releases the pad
    PinInfo info(int pin) const;
    uint64_t version() const { return version_; }

    bool trace = true;
    std::function<void(bool level)> irq;  // GPIO interrupt source (CPU)

private:
    class IoMux : public Peripheral {
    public:
        explicit IoMux(Gpio& g) : Peripheral("IO_MUX"), g_(g) {}
        uint32_t read(uint32_t off) override;
        void write(uint32_t off, uint32_t value) override;

    private:
        Gpio& g_;
    };

    bool via_matrix(int pin) const;
    int chip_drive(int pin, std::string* function) const;  // -1 = not driving
    int pad_level(int pin) const;
    void update();  // recompute input levels, interrupts, trace

    const GpioPads& pads_;
    const int pins_;
    const uint32_t pin_mask_;
    IoMux iomux_;
    uint32_t out_ = 0, enable_ = 0, strap_ = 0, status_ = 0;
    uint32_t in_ = 0;                       // last resolved GPIO_IN
    std::array<uint32_t, kMaxPins> pin_cfg_{};  // GPIO_PINn
    std::array<uint32_t, kMaxPins> out_sel_{};  // GPIO_FUNCn_OUT_SEL_CFG
    std::array<uint32_t, 128> in_sel_{};     // GPIO_FUNCn_IN_SEL_CFG (by signal)
    std::array<uint32_t, kMaxPins> mux_{};      // IO_MUX_GPIOn
    uint32_t mux_ctrl_ = 0;
    std::array<int, kMaxPins> external_{};
    std::array<int, kMaxPins> traced_{};
    uint64_t version_ = 0;
};

}  // namespace esp32sim::c3
