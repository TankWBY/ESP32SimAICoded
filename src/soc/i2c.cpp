#include "soc/i2c.hpp"

#include <cstdlib>

#include "util/log.hpp"

namespace esp32sim {

// ---------------------------------------------------------------- devices

bool I2cEeprom::start(bool read) {
    addr_phase_ = !read;
    return true;
}

bool I2cEeprom::write(uint8_t byte) {
    if (addr_phase_) {
        ptr_ = byte % mem_.size();
        addr_phase_ = false;
    } else {
        mem_[ptr_] = byte;
        ptr_ = (ptr_ + 1) % mem_.size();
    }
    return true;
}

uint8_t I2cEeprom::read() {
    uint8_t v = mem_[ptr_];
    ptr_ = (ptr_ + 1) % mem_.size();
    return v;
}

std::unique_ptr<I2cDevice> make_i2c_device(const std::string& spec, int* address, std::string* error) {
    const size_t at = spec.find('@');
    const std::string kind = spec.substr(0, at);
    char* end = nullptr;
    const long addr = at == std::string::npos ? -1 : std::strtol(spec.c_str() + at + 1, &end, 0);
    if (addr < 0x08 || addr > 0x77 || (end && *end)) {
        *error = "I2C device '" + spec + "': expected KIND@ADDRESS with a 7-bit address, e.g. eeprom@0x50";
        return nullptr;
    }
    *address = int(addr);
    if (kind == "eeprom") return std::make_unique<I2cEeprom>();
    *error = "unknown I2C device kind '" + kind + "' (known: eeprom)";
    return nullptr;
}

// ---------------------------------------------------------------- controller

namespace {
constexpr uint32_t CTR = 0x04, SR = 0x08, FIFO_CONF = 0x18, DATA = 0x1C, INT_RAW = 0x20, INT_CLR = 0x24,
                   INT_ENA = 0x28, INT_STATUS = 0x2C, COMD0 = 0x58, DATE = 0xF8;
constexpr uint32_t CTR_TRANS_START = 1u << 5, CTR_FSM_RST = 1u << 10, CTR_CONF_UPGATE = 1u << 11;
constexpr uint32_t RX_FIFO_RST = 1u << 12, TX_FIFO_RST = 1u << 13;
constexpr uint32_t INT_END_DETECT = 1u << 3, INT_TRANS_COMPLETE = 1u << 7, INT_NACK = 1u << 10,
                   INT_TXFIFO_WM = 1u << 1;
constexpr uint32_t CMD_DONE = 1u << 31, CMD_ACK_CHECK = 1u << 8, CMD_ACK_EXP = 1u << 9;
enum { OP_WRITE = 1, OP_STOP = 2, OP_READ = 3, OP_END = 4, OP_RSTART = 6 };
constexpr size_t kFifo = 32;
}  // namespace

void I2cController::reset() {
    regs_.fill(0);
    tx_.clear();
    rx_.clear();
    cmd_.fill(0);
    cur_ = nullptr;
    addressing_ = nack_ = false;
    int_raw_ = 0;
    update_irq();
}

void I2cController::update_irq() {
    if (irq) irq((int_raw_ & regs_[INT_ENA / 4]) != 0);
}

uint32_t I2cController::read(uint32_t off) {
    if (off >= COMD0 && off < COMD0 + 32) return cmd_[(off - COMD0) / 4];
    switch (off) {
    case SR: return (nack_ ? 1u : 0u) | uint32_t(rx_.size()) << 8 | uint32_t(tx_.size()) << 18;
    case DATA: {
        if (rx_.empty()) return 0;
        uint8_t b = rx_.front();
        rx_.pop_front();
        return b;
    }
    case INT_RAW: return int_raw_;
    case INT_STATUS: return int_raw_ & regs_[INT_ENA / 4];
    case DATE: return 0x2201060;
    default: return off < 0x100 ? regs_[off / 4] : 0;
    }
}

void I2cController::write(uint32_t off, uint32_t value) {
    if (off >= COMD0 && off < COMD0 + 32) {
        cmd_[(off - COMD0) / 4] = value;
        return;
    }
    switch (off) {
    case CTR:
        regs_[CTR / 4] = value & ~(CTR_TRANS_START | CTR_FSM_RST | CTR_CONF_UPGATE);
        if (value & CTR_FSM_RST) cur_ = nullptr, addressing_ = false;
        if (value & CTR_TRANS_START) run();
        return;
    case FIFO_CONF:
        if (value & RX_FIFO_RST) rx_.clear();
        if (value & TX_FIFO_RST) tx_.clear();
        regs_[FIFO_CONF / 4] = value;
        return;
    case DATA:
        if (tx_.size() < kFifo) tx_.push_back(uint8_t(value));
        return;
    case INT_CLR:
        int_raw_ &= ~value;
        update_irq();
        return;
    case INT_ENA:
        regs_[INT_ENA / 4] = value;
        update_irq();
        return;
    default:
        if (off < 0x100) regs_[off / 4] = value;
        return;
    }
}

// Executes the command list from COMD0 until STOP, END or a NACK.
void I2cController::run() {
    for (auto& c : cmd_) {
        const int op = int((c >> 11) & 7);
        const uint32_t n = c & 0xFF;
        switch (op) {
        case OP_RSTART:
            if (cur_) cur_->stop();
            cur_ = nullptr;
            addressing_ = true;
            break;
        case OP_WRITE:
            for (uint32_t i = 0; i < n; ++i) {
                uint8_t b = 0xFF;
                if (!tx_.empty()) b = tx_.front(), tx_.pop_front();
                bool ack;
                if (addressing_) {
                    addressing_ = false;
                    auto it = devices_.find(b >> 1);
                    cur_ = it == devices_.end() ? nullptr : it->second;
                    ack = cur_ && cur_->start(b & 1);
                    if (!ack) cur_ = nullptr;
                } else {
                    ack = cur_ && cur_->write(b);
                }
                nack_ = !ack;
                if ((c & CMD_ACK_CHECK) && nack_ != ((c & CMD_ACK_EXP) != 0)) {
                    c |= CMD_DONE;
                    int_raw_ |= INT_NACK;
                    update_irq();
                    return;  // the controller stops; software resets it
                }
            }
            break;
        case OP_READ:
            for (uint32_t i = 0; i < n; ++i) {
                uint8_t b = cur_ ? cur_->read() : 0xFF;
                if (rx_.size() < kFifo) rx_.push_back(b);
            }
            break;
        case OP_STOP:
            if (cur_) cur_->stop();
            cur_ = nullptr;
            c |= CMD_DONE;
            int_raw_ |= INT_TRANS_COMPLETE;
            update_irq();
            return;
        case OP_END:
            c |= CMD_DONE;
            int_raw_ |= INT_END_DETECT | INT_TXFIFO_WM;
            update_irq();
            return;  // paused; the next TRANS_START continues the transaction
        default:
            LOGW("i2c: unknown command op %d", op);
            return;
        }
        c |= CMD_DONE;
    }
}

}  // namespace esp32sim
