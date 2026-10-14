// ESP32-C6 pad table for the shared GPIO/IO_MUX model.
#pragma once
#include "soc/esp32c3/gpio.hpp"

namespace esp32sim::c6 {
extern const c3::GpioPads kPads;
extern const c3::SignalName kOutSignals[];
extern const size_t kOutSignalCount;
}  // namespace esp32sim::c6
