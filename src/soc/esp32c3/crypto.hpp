#pragma once
// Crypto accelerators of the RISC-V ESP32 chips (same layout on the C3 and C6).
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

// AES accelerator: AES-128/256, CPU-driven ("typical") mode for one block
// and DMA mode for ECB/CBC/OFB/CTR/CFB8/CFB128 over many blocks, with its
// "done" interrupt. Operations complete instantly.
class Aes : public Peripheral {
public:
    explicit Aes(std::string name) : Peripheral(std::move(name)) { reset(); }
    uint32_t read(uint32_t off) override;
    void write(uint32_t off, uint32_t value) override;
    void reset() override;

    // DMA mode: pull input bytes from the GDMA TX link, push results to the RX link.
    std::function<size_t(uint8_t* dst, size_t len)> dma_read;
    std::function<void(const uint8_t* src, size_t len)> dma_write;
    std::function<void(bool level)> irq;

private:
    void run_typical();
    void run_dma();
    void encrypt_block(const uint8_t in[16], uint8_t out[16]) const;
    void decrypt_block(const uint8_t in[16], uint8_t out[16]) const;
    void expand_key();

    std::array<uint32_t, 8> key_{};
    std::array<uint32_t, 4> text_in_{}, text_out_{}, iv_{};
    uint32_t mode_ = 0, state_ = 0, dma_enable_ = 0, block_mode_ = 0, block_num_ = 0, inc_sel_ = 0;
    uint32_t int_ena_ = 0;
    bool int_raw_ = false;
    std::array<uint8_t, 240> round_keys_{};
    int rounds_ = 10;
};

}  // namespace esp32sim::c3
