#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>

namespace esp32sim {

// A memory-mapped peripheral. Accesses are 32-bit, offset relative to base.
class Peripheral {
public:
    explicit Peripheral(std::string name) : name_(std::move(name)) {}
    virtual ~Peripheral() = default;
    virtual uint32_t read(uint32_t offset) = 0;
    virtual void write(uint32_t offset, uint32_t value) = 0;
    virtual void reset() {}
    const std::string& name() const { return name_; }

private:
    std::string name_;
};

// Plain register storage: reads return what was written (or a default).
// Used for blocks whose behaviour does not matter yet, so that init code
// doing read-modify-write sequences keeps working.
class RegisterFile : public Peripheral {
public:
    using Peripheral::Peripheral;
    uint32_t read(uint32_t offset) override {
        auto it = regs_.find(offset);
        if (it != regs_.end()) return it->second;
        auto d = defaults_.find(offset);
        return d == defaults_.end() ? 0 : d->second;
    }
    void write(uint32_t offset, uint32_t value) override { regs_[offset] = value; }
    void reset() override { regs_.clear(); }
    void set_default(uint32_t offset, uint32_t value) { defaults_[offset] = value; }
    uint32_t read_const(uint32_t offset) const {
        auto it = regs_.find(offset);
        if (it != regs_.end()) return it->second;
        auto d = defaults_.find(offset);
        return d == defaults_.end() ? 0 : d->second;
    }

protected:
    std::unordered_map<uint32_t, uint32_t> regs_;
    std::unordered_map<uint32_t, uint32_t> defaults_;
};

}  // namespace esp32sim
