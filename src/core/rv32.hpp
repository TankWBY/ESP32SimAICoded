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
    explicit Rv32Cpu(Bus& bus);

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

    uint32_t x[33] = {};  // x[32] is a scratch sink for writes to x0
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

    // Drop translated code. Must be called when code memory changes behind the
    // CPU's back (loader writes, flash writes, MMU remaps). CPU stores to
    // translated code are caught by the bus code map (Bus::on_code_write).
    void flush_icache();

    // Stop run() before the next instruction (e.g. a peripheral reset the chip).
    void request_stop() { stop_ = true; attention_ = true; }

private:
    // Code is translated into basic blocks: runs of pre-decoded instructions
    // that end at a jump, a branch or an instruction handled by the generic
    // slow path (CSR, AMO, SYSTEM). A block executes without per-instruction
    // bookkeeping; time and pc are only materialised when something can see
    // them (MMIO, traps, the end of the block).
    struct Op {
        uint8_t op, rd, rs1, rs2;
        uint32_t imm;  // immediate, precomputed target, or the raw encoding for slow ops
        uint32_t pc;
    };
    struct Block {
        uint32_t pc = 1;     // tag (odd = invalid)
        uint32_t count = 0;  // instructions; ops[count] is an end marker holding the next pc
        Op* ops = nullptr;
    };
    static constexpr uint32_t kBlockBits = 14, kMaxBlockOps = 64, kArenaOps = 1u << 18;

    uint64_t core_acc_ = 0, core_mark_ = 0;
    // Translate the block at pc into the cache, or, with max_ops below
    // kMaxBlockOps, a short uncached one for the end of a run() budget.
    const Block* translate(uint32_t pc, uint32_t max_ops);
    void decode(uint32_t insn, uint32_t len, uint32_t raw, Op& o, bool& ends_block);
    void trap(uint32_t cause, uint32_t tval);
    bool csr_access(uint32_t csr, uint32_t* old, uint32_t newval, bool do_write);
    bool check_interrupts();
    void execute(uint32_t insn, uint32_t len);

    Bus& bus_;
    std::unique_ptr<Block[]> blocks_{new Block[1u << kBlockBits]};
    std::unique_ptr<Op[]> arena_{new Op[kArenaOps]};
    uint32_t arena_used_ = 0;
    Block partial_;
    std::unique_ptr<Op[]> partial_ops_{new Op[kMaxBlockOps + 1]};
    uint32_t lr_addr_ = 0;
    bool lr_valid_ = false;
    bool stop_ = false;
    bool attention_ = true;  // re-check halted/irq/wfi state before the next instruction
};

}  // namespace esp32sim
