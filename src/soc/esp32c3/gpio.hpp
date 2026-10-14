#pragma once
// ESP32-C3 GPIO: the GPIO block (0x6000_4000: output/enable/input registers,
// per-pin interrupt config, the GPIO matrix) and IO_MUX (0x6000_9000: pad
// function, pulls, input enable). Pads have a resolved level that combines
// what the chip drives, the pulls, and an external drive set by the UI.
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
extern const SignalName kOutSignals[];
extern const size_t kOutSignalCount;
extern const SignalName kInSignals[];
extern const size_t kInSignalCount;

class Gpio : public Peripheral {
public:
    static constexpr int kPins = 22;
    explicit Gpio(std::string name);
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

    IoMux iomux_;
    uint32_t out_ = 0, enable_ = 0, strap_ = 0, status_ = 0;
    uint32_t in_ = 0;                       // last resolved GPIO_IN
    std::array<uint32_t, kPins> pin_cfg_{};  // GPIO_PINn
    std::array<uint32_t, kPins> out_sel_{};  // GPIO_FUNCn_OUT_SEL_CFG
    std::array<uint32_t, 128> in_sel_{};     // GPIO_FUNCn_IN_SEL_CFG (by signal)
    std::array<uint32_t, kPins> mux_{};      // IO_MUX_GPIOn
    uint32_t mux_ctrl_ = 0;
    std::array<int, kPins> external_{};
    std::array<int, kPins> traced_{};
    uint64_t version_ = 0;
};

}  // namespace esp32sim::c3
