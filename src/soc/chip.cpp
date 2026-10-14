#include "soc/chip.hpp"

#include "soc/esp32c3/esp32c3.hpp"
#include "soc/esp32c6/esp32c6.hpp"

namespace esp32sim {

const std::vector<ChipInfo>& chip_catalog() {
    static const std::vector<ChipInfo> catalog = {
        {"esp32c3", "ESP32-C3", "RISC-V RV32IMC", true, "runs ESP-IDF and Arduino apps with the ROM image: CPU, interrupts, UART, SPI flash, MMU, timers, watchdogs, GPIO, SHA, AES, GDMA, SPI2, I2C, LEDC, ADC, USB-Serial-JTAG"},
        {"esp32c6", "ESP32-C6", "RISC-V RV32IMAC", true, "runs ESP-IDF and Arduino apps with the ROM image: CPU, interrupts (PLIC), UART, SPI flash, MMU, timers, watchdogs, GPIO, SHA, AES, GDMA, SPI2, I2C, LEDC, ADC, USB-Serial-JTAG; no radios or LP core"},
        {"esp32h2", "ESP32-H2", "RISC-V RV32IMAC", false, "planned"},
        {"esp32", "ESP32", "Xtensa LX6 (dual core)", false, "planned: needs Xtensa core"},
        {"esp32s2", "ESP32-S2", "Xtensa LX7", false, "planned: needs Xtensa core"},
        {"esp32s3", "ESP32-S3", "Xtensa LX7 (dual core)", false, "planned: needs Xtensa core"},
    };
    return catalog;
}

std::unique_ptr<Chip> create_chip(const ChipConfig& cfg, std::string* error) {
    if (cfg.variant == "esp32c3") return std::make_unique<Esp32c3>(cfg);
    if (cfg.variant == "esp32c6") return std::make_unique<Esp32c6>(cfg);
    for (const auto& c : chip_catalog()) {
        if (c.mcu == cfg.variant) {
            if (error) *error = c.name + " is not emulated yet (" + c.note + ")";
            return nullptr;
        }
    }
    if (error) *error = "unknown chip '" + cfg.variant + "'";
    return nullptr;
}

void AutoResetCircuit::set_lines(bool dtr, bool rts, double now) {
    dtr_ = dtr;
    rts_ = rts;
    bool en_low = rts_ && !dtr_;
    if (en_low) {
        en_low_ = true;
        pending_release_ = false;
    } else if (en_low_ && !pending_release_) {
        // EN starts charging; the chip leaves reset once the RC delay passes.
        pending_release_ = true;
        release_at_ = now + delay_;
    }
}

bool AutoResetCircuit::poll(double now, BootMode* mode) {
    if (!pending_release_ || now < release_at_) return false;
    pending_release_ = false;
    en_low_ = false;
    bool boot_low = dtr_ && !rts_;
    *mode = boot_low ? BootMode::Download : BootMode::Flash;
    return true;
}

}  // namespace esp32sim
