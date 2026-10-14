#pragma once
// I2C master controller of the RISC-V ESP32 chips (same register layout on
// the C3 and C6), at the transaction level: the command list (RSTART, WRITE,
// READ, STOP, END) runs to completion as soon as software starts it, against
// virtual devices on the bus. An address nobody answers is NACKed, as on a
// real bus with nothing connected.
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "soc/peripheral.hpp"

namespace esp32sim {

// A device on the emulated I2C bus.
class I2cDevice {
public:
    virtual ~I2cDevice() = default;
    // Addressed after a (repeated) START; return true to ACK.
    virtual bool start(bool read) = 0;
    virtual bool write(uint8_t byte) = 0;  // true = ACK
    virtual uint8_t read() = 0;
    virtual void stop() {}
};

// A 24C02-style EEPROM: 256 bytes, one address byte, sequential reads and
// writes that wrap around.
class I2cEeprom : public I2cDevice {
public:
    explicit I2cEeprom(size_t size = 256) : mem_(size, 0xFF) {}
    bool start(bool read) override;
    bool write(uint8_t byte) override;
    uint8_t read() override;

private:
    std::vector<uint8_t> mem_;
    size_t ptr_ = 0;
    bool addr_phase_ = false;
};

// Builds a device from a "--i2c-dev" spec such as "eeprom@0x50".
// Returns null and sets *error for an unknown spec.
std::unique_ptr<I2cDevice> make_i2c_device(const std::string& spec, int* address, std::string* error);

class I2cController : public Peripheral {
public:
    explicit I2cController(std::string name) : Peripheral(std::move(name)) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;

    void attach(int address, I2cDevice* dev) { devices_[address] = dev; }
    std::function<void(bool level)> irq;

private:
    void run();
    void update_irq();
    std::map<int, I2cDevice*> devices_;
    std::array<uint32_t, 0x100 / 4> regs_{};
    std::deque<uint8_t> tx_, rx_;
    std::array<uint32_t, 8> cmd_{};
    I2cDevice* cur_ = nullptr;
    bool addressing_ = false;  // next written byte is an address
    bool nack_ = false;        // last byte was not acknowledged
    uint32_t int_raw_ = 0;
};

}  // namespace esp32sim
