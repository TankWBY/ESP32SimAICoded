// Chip-variant abstraction. Each emulated ESP32 family member implements Chip;
// the host side (serial transports, board reset circuit, main loop) only talks
// to this interface.
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace esp32sim {

enum class BootMode { Flash, Download };

// What a GPIO pad is doing right now, for the UI.
struct PinInfo {
    int gpio = 0;
    std::string function;      // "GPIO", "U0TXD", "LEDC_LS_SIG_OUT0", "SPI flash", ...
    bool output = false;       // chip output driver enabled
    bool input = false;        // input buffer enabled (the code can read the pin)
    bool pull_up = false, pull_down = false, open_drain = false;
    int chip_level = -1;       // level the chip drives, -1 when not driving
    int external = -1;         // level forced from outside (UI), -1 when released
    int level = -1;            // resulting pad level, -1 = floating
    std::string interrupt;     // "", "rising", "falling", "any edge", "low level", "high level"
};

struct ChipConfig {
    std::string variant = "esp32c3";        // PlatformIO "build.mcu" naming
    uint32_t flash_size = 4 * 1024 * 1024;
    std::string flash_path = "flash.bin";
    uint32_t cpu_hz = 160000000;
    std::array<uint8_t, 6> mac = {0x7C, 0xDF, 0xA1, 0x5E, 0x00, 0x01};
    bool trace_gpio = true;
    bool usb_console = false;              // serial endpoints carry USB-Serial-JTAG instead of UART0
    std::string rom_path;                  // Espressif ROM ELF (esp-rom-elfs); empty = HLE only
};

class Chip {
public:
    virtual ~Chip() = default;

    virtual std::string name() const = 0;          // e.g. "ESP32-C3"
    virtual void reset(BootMode mode) = 0;         // EN released with given strap
    virtual void hold_in_reset() = 0;              // EN low: chip stops
    virtual void run_for(uint64_t cycles) = 0;     // advance emulated time
    virtual void serial_input(const uint8_t* data, size_t len) = 0;  // host -> UART0 RX
    virtual uint32_t cpu_hz() const = 0;
    virtual void shutdown() = 0;                    // flush flash to disk
    virtual bool idle() const = 0;                  // nothing running (loader waiting / halted)
    virtual uint64_t instructions() const = 0;      // retired instruction count
    virtual std::string where() const { return {}; }  // current pc, for diagnostics
    // "running", "download mode", "held in reset" or "halted: <why>".
    virtual std::string status() const { return idle() ? "idle" : "running"; }
    virtual void add_symbols(const std::string& elf_path) { (void)elf_path; }  // for diagnostics

    // GPIO pads. Pins usable on a board are a subset (flash pins excluded).
    virtual std::vector<int> user_gpios() const { return {}; }
    virtual PinInfo pin_info(int gpio) const { PinInfo p; p.gpio = gpio; return p; }
    // Drive a pad from outside: 0 or 1, or -1 to release it.
    virtual void set_pin_input(int gpio, int level) { (void)gpio; (void)level; }
    // Changes whenever any pad state changes (cheap change detection).
    virtual uint64_t pins_version() const { return 0; }

    // Bytes the chip transmits on its serial console (UART0 TX).
    std::function<void(const uint8_t*, size_t)> serial_output;
};

struct ChipInfo {
    std::string mcu;        // "esp32c3"
    std::string name;       // "ESP32-C3"
    std::string core;       // "RISC-V RV32IMC"
    bool supported;
    std::string note;
};

const std::vector<ChipInfo>& chip_catalog();
std::unique_ptr<Chip> create_chip(const ChipConfig& cfg, std::string* error);

// Classic USB-UART auto-reset circuit found on ESP32 dev boards:
// two transistors cross-wired so that RTS drives EN and DTR drives the boot
// strapping pin (GPIO0 on ESP32, GPIO9 on ESP32-C3). The RC delay on EN is
// emulated so the usual esptool DTR/RTS dance selects download mode. It is
// longer than on real boards because over RFC 2217 every line change is a
// network round trip, so the transient DTR=RTS=1 state lasts tens of ms.
class AutoResetCircuit {
public:
    explicit AutoResetCircuit(double en_delay_s = 0.1) : delay_(en_delay_s) {}
    // Feed new line states; returns true when the chip should be held in reset.
    void set_lines(bool dtr, bool rts, double now);
    // Poll from the main loop. Returns true when EN has been released and the
    // chip should boot now (mode filled in).
    bool poll(double now, BootMode* mode);
    bool in_reset() const { return en_low_; }

private:
    double delay_;
    bool dtr_ = false, rts_ = false;
    bool en_low_ = false;
    bool pending_release_ = false;
    double release_at_ = 0;
};

}  // namespace esp32sim
