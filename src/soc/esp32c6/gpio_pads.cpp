#include "soc/esp32c6/gpio_pads.hpp"

namespace esp32sim::c6 {

namespace {
constexpr uint32_t FUN_WPU = 1u << 8, FUN_IE = 1u << 9;

// IO_MUX function 0 and 2 of each pad (ESP32-C6 TRM, IO MUX functions).
// Function 1 is always the GPIO matrix.
const char* const kFunc0[31] = {"GPIO0",  "GPIO1",  "GPIO2",     "GPIO3",     "MTMS",      "MTDI",      "MTCK",
                                "MTDO",   "GPIO8",  "GPIO9",     "GPIO10",    "GPIO11",    "GPIO12",    "GPIO13",
                                "GPIO14", "GPIO15", "U0TXD",     "U0RXD",     "SDIO_CMD",  "SDIO_CLK",  "SDIO_DATA0",
                                "SDIO_DATA1", "SDIO_DATA2", "SDIO_DATA3", "SPICS0", "SPIQ", "SPIWP", "GPIO27",
                                "SPIHD",  "SPICLK", "SPID"};
const char* const kFunc2[31] = {nullptr,   nullptr,   "FSPIQ",   nullptr,   "FSPIHD",  "FSPIWP",  "FSPICLK", "FSPID",
                                nullptr,   nullptr,   nullptr,   nullptr,   nullptr,   nullptr,   nullptr,   nullptr,
                                "FSPICS0", "FSPICS1", "FSPICS2", "FSPICS3", "FSPICS4", "FSPICS5"};

uint32_t default_mux(int pin) {
    uint32_t v = 2u << 10;  // FUN_DRV = 2
    bool ie = pin != 16;    // every input buffer is on at reset except U0TXD's
    if (ie) v |= FUN_IE;
    if (pin == 9) v |= FUN_WPU;                 // boot strap pull-up
    if (pin >= 24 && pin <= 30) v |= FUN_WPU;   // SPI flash
    if (pin == 13) v |= FUN_WPU;                // USB D+ pull-up
    return v;
}
}  // namespace

const c3::GpioPads kPads = {31, kFunc0, kFunc2, default_mux, 0x7F000000 /* GPIO24-30 */, 12, 13, 16,
                            kOutSignals, kOutSignalCount, 0x2201120};

}  // namespace esp32sim::c6
