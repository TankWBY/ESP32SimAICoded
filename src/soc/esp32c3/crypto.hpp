#pragma once
// ESP32-C3 crypto accelerators.
#include <array>
#include <cstdint>
#include <functional>

#include "soc/peripheral.hpp"

namespace esp32sim::c3 {

// SHA accelerator (SHA-1/224/256), CPU-driven ("typical SHA") mode. H_MEM and
// M_MEM hold big-endian words, so the bytes software writes or reads appear
// in message/digest order. DMA mode reads its input via `dma_read`.
class Sha : public Peripheral {
public:
    explicit Sha(std::string name) : Peripheral(std::move(name)) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;

    // DMA mode hook: fetch the next 64-byte block from the GDMA out-link.
    std::function<bool(uint8_t* block)> dma_read;

private:
    void process(bool first);
    void compress();
    uint32_t mode_ = 0, block_num_ = 0, int_ena_ = 0;
    std::array<uint32_t, 8> h_{};   // big-endian state as stored in H_MEM
    std::array<uint32_t, 16> m_{};  // message block as stored in M_MEM
};

}  // namespace esp32sim::c3
