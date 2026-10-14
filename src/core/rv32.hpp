// RV32IMAC + Zicsr interpreter (machine mode only), as used by the ESP32-C3
// (RV32IMC), ESP32-C6/H2 (RV32IMAC) and similar Espressif RISC-V parts.
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "core/bus.hpp"

namespace esp32sim {

class Rv32Cpu {
public:
    explicit Rv32Cpu(Bus& bus) : bus_(bus) {}

    void reset(uint32_t pc);

    // Execute up to max_instructions. Returns the number actually executed.
    // Stops early when the core halts (fatal trap, WFI with nothing pending,
    // or a hook asked to stop).
    uint64_t run(uint64_t max_instructions);

    // Raise or clear a machine interrupt request. `cause` is the CPU
    // interrupt number (1..31); the SoC interrupt matrix decides which.
    void set_irq(int cause, bool level);
    // Replace all interrupt request lines at once (SoC interrupt controllers).
    void set_irq_lines(uint32_t lines);
    // Interrupt lines that are taken without a matching mie bit. The ESP32-C3
    // gates its 31 interrupt lines in the interrupt matrix, not in mie.
    uint32_t mie_bypass = 0;

    uint32_t x[32] = {};
    uint32_t pc = 0;
    // `cycles` is the SoC time base (ticks of the fastest CPU clock). Each
    // instruction costs `cycle_cost` ticks, so a core clocked at a quarter of
    // that rate has cycle_cost 4. mcycle and friends count core cycles.
    uint64_t cycles = 0;
    uint64_t instret = 0;
    uint32_t cycle_cost = 1;
    void set_cycle_cost(uint32_t cost);
    uint64_t core_cycles() const { return core_acc_ + (cycles - core_mark_) / cycle_cost; }

    // Machine CSRs.
    uint32_t mstatus = 0, mie = 0, mip = 0, mtvec = 0, mscratch = 0;
    uint32_t mepc = 0, mcause = 0, mtval = 0;
    uint32_t misa = (1u << 30) | (1u << 8) /*I*/ | (1u << 12) /*M*/ | (1u << 0) /*A*/ | (1u << 2) /*C*/;

    bool halted = false;       // fatal stop; needs reset
    bool waiting = false;      // WFI, resumes on interrupt
    std::string halt_reason;

    // Called before executing an instruction whose address lies inside
    // [hook_lo, hook_hi). Used for high-level emulation of mask ROM routines.
    // Return true if the hook handled the call (it must update pc).
    std::function<bool(Rv32Cpu&)> hook;
    uint32_t hook_lo = 0, hook_hi = 0;

    // Vendor/unknown CSR access is forwarded here; return false to raise an
    // illegal-instruction exception.
    std::function<bool(uint32_t csr, uint32_t* value, bool write)> custom_csr;

    // Trap with no handler installed (mtvec == 0): report and halt.
    std::function<void(Rv32Cpu&, uint32_t cause, uint32_t tval)> on_fatal_trap;

    static int expand_compressed(uint16_t c, uint32_t* out);  // returns 0 if illegal

    // Drop cached decodes. Must be called when code memory changes behind the
    // CPU's back (loader writes, flash writes, MMU remaps). CPU stores
    // invalidate the affected entries themselves.
    void flush_icache();

    // Stop run() before the next instruction (e.g. a peripheral reset the chip).
    void request_stop() { stop_ = true; attention_ = true; }

    // Pre-decoded instruction, cached per address.
    struct Decoded {
        uint32_t pc = 1;     // tag (odd = invalid)
        uint32_t raw = 0;    // original encoding, to detect modified code
        uint16_t op = 0;
        uint8_t rd = 0, rs1 = 0, rs2 = 0, len = 0;
        int32_t imm = 0;
        uint32_t insn = 0;   // expanded 32-bit encoding (slow path)
    };

private:
    uint64_t core_acc_ = 0, core_mark_ = 0;
    void decode(uint32_t insn, uint32_t len, uint32_t raw, Decoded& d);
    void exec(const Decoded& d);
    void trap(uint32_t cause, uint32_t tval);
    bool csr_access(uint32_t csr, uint32_t* old, uint32_t newval, bool do_write);
    bool check_interrupts();
    void execute(uint32_t insn, uint32_t len);

    Bus& bus_;
    static constexpr uint32_t kCacheBits = 15;
    std::unique_ptr<Decoded[]> icache_{new Decoded[1u << kCacheBits]};
    uint32_t lr_addr_ = 0;
    bool lr_valid_ = false;
    bool stop_ = false;
    bool attention_ = true;  // re-check halted/irq/wfi state before the next instruction
    void invalidate(uint32_t addr) {
        const uint32_t mask = (1u << kCacheBits) - 1;
        icache_[(addr >> 1) & mask].pc = 1;
        icache_[((addr >> 1) + 1) & mask].pc = 1;
        icache_[((addr - 2) >> 1) & mask].pc = 1;
    }
};

}  // namespace esp32sim
