#include "core/rv32.hpp"

namespace esp32sim {

namespace {

inline int32_t sext(uint32_t v, int bits) {
    return int32_t(v << (32 - bits)) >> (32 - bits);
}
inline uint32_t bits(uint32_t v, int hi, int lo) {
    return (v >> lo) & ((1u << (hi - lo + 1)) - 1);
}
inline uint32_t bit(uint32_t v, int b) { return (v >> b) & 1u; }

// 32-bit encoders used to expand compressed instructions.
inline uint32_t enc_i(uint32_t op, uint32_t rd, uint32_t f3, uint32_t rs1, int32_t imm) {
    return (uint32_t(imm & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
}
inline uint32_t enc_s(uint32_t op, uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
    uint32_t u = uint32_t(imm);
    return (bits(u, 11, 5) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (bits(u, 4, 0) << 7) | op;
}
inline uint32_t enc_b(uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
    uint32_t u = uint32_t(imm);
    return (bit(u, 12) << 31) | (bits(u, 10, 5) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) |
           (bits(u, 4, 1) << 8) | (bit(u, 11) << 7) | 0x63;
}
inline uint32_t enc_j(uint32_t rd, int32_t imm) {
    uint32_t u = uint32_t(imm);
    return (bit(u, 20) << 31) | (bits(u, 10, 1) << 21) | (bit(u, 11) << 20) | (bits(u, 19, 12) << 12) |
           (rd << 7) | 0x6F;
}
inline uint32_t enc_r(uint32_t f7, uint32_t rs2, uint32_t rs1, uint32_t f3, uint32_t rd) {
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | 0x33;
}

enum Cause : uint32_t {
    kInsnMisaligned = 0,
    kIllegal = 2,
    kBreakpoint = 3,
    kLoadMisaligned = 4,
    kStoreMisaligned = 6,
    kEcallM = 11,
};

constexpr uint32_t MSTATUS_MIE = 1u << 3;
constexpr uint32_t MSTATUS_MPIE = 1u << 7;
constexpr uint32_t MSTATUS_MPP = 3u << 11;

}  // namespace

int Rv32Cpu::expand_compressed(uint16_t c, uint32_t* out) {
    const uint32_t op = c & 3, f3 = bits(c, 15, 13);
    const uint32_t rdp = 8 + bits(c, 4, 2), rs1p = 8 + bits(c, 9, 7);
    const uint32_t rd = bits(c, 11, 7), rs2 = bits(c, 6, 2);
    switch (op) {
    case 0:
        switch (f3) {
        case 0: {  // C.ADDI4SPN
            uint32_t imm = (bits(c, 12, 11) << 4) | (bits(c, 10, 7) << 6) | (bit(c, 6) << 2) | (bit(c, 5) << 3);
            if (!imm) return 0;
            *out = enc_i(0x13, rdp, 0, 2, int32_t(imm));
            return 1;
        }
        case 2: {  // C.LW
            uint32_t imm = (bits(c, 12, 10) << 3) | (bit(c, 6) << 2) | (bit(c, 5) << 6);
            *out = enc_i(0x03, rdp, 2, rs1p, int32_t(imm));
            return 1;
        }
        case 6: {  // C.SW
            uint32_t imm = (bits(c, 12, 10) << 3) | (bit(c, 6) << 2) | (bit(c, 5) << 6);
            *out = enc_s(0x23, 2, rs1p, rdp, int32_t(imm));
            return 1;
        }
        }
        return 0;
    case 1:
        switch (f3) {
        case 0:  // C.ADDI / C.NOP
            *out = enc_i(0x13, rd, 0, rd, sext((bit(c, 12) << 5) | rs2, 6));
            return 1;
        case 1:    // C.JAL (RV32)
        case 5: {  // C.J
            uint32_t imm = (bit(c, 12) << 11) | (bit(c, 11) << 4) | (bits(c, 10, 9) << 8) | (bit(c, 8) << 10) |
                           (bit(c, 7) << 6) | (bit(c, 6) << 7) | (bits(c, 5, 3) << 1) | (bit(c, 2) << 5);
            *out = enc_j(f3 == 1 ? 1 : 0, sext(imm, 12));
            return 1;
        }
        case 2:  // C.LI
            *out = enc_i(0x13, rd, 0, 0, sext((bit(c, 12) << 5) | rs2, 6));
            return 1;
        case 3:
            if (rd == 2) {  // C.ADDI16SP
                uint32_t imm = (bit(c, 12) << 9) | (bit(c, 6) << 4) | (bit(c, 5) << 6) | (bits(c, 4, 3) << 7) |
                               (bit(c, 2) << 5);
                if (!imm) return 0;
                *out = enc_i(0x13, 2, 0, 2, sext(imm, 10));
            } else {  // C.LUI
                uint32_t imm = (bit(c, 12) << 17) | (rs2 << 12);
                if (!imm) return 0;
                *out = (uint32_t(sext(imm, 18)) & 0xFFFFF000u) | (rd << 7) | 0x37;
            }
            return 1;
        case 4: {
            uint32_t f2 = bits(c, 11, 10);
            uint32_t shamt = rs2;
            if (f2 == 0) {  // C.SRLI
                if (bit(c, 12)) return 0;
                *out = enc_i(0x13, rs1p, 5, rs1p, int32_t(shamt));
            } else if (f2 == 1) {  // C.SRAI
                if (bit(c, 12)) return 0;
                *out = enc_i(0x13, rs1p, 5, rs1p, int32_t(shamt | 0x400));
            } else if (f2 == 2) {  // C.ANDI
                *out = enc_i(0x13, rs1p, 7, rs1p, sext((bit(c, 12) << 5) | rs2, 6));
            } else {
                if (bit(c, 12)) return 0;
                static const uint32_t f3s[4] = {0, 4, 6, 7};  // SUB XOR OR AND
                uint32_t sel = bits(c, 6, 5);
                *out = enc_r(sel == 0 ? 0x20 : 0, rdp, rs1p, f3s[sel], rs1p);
            }
            return 1;
        }
        case 6:
        case 7: {  // C.BEQZ / C.BNEZ
            uint32_t imm = (bit(c, 12) << 8) | (bits(c, 11, 10) << 3) | (bits(c, 6, 5) << 6) | (bits(c, 4, 3) << 1) |
                           (bit(c, 2) << 5);
            *out = enc_b(f3 == 6 ? 0 : 1, rs1p, 0, sext(imm, 9));
            return 1;
        }
        }
        return 0;
    case 2:
        switch (f3) {
        case 0:  // C.SLLI
            if (bit(c, 12)) return 0;
            *out = enc_i(0x13, rd, 1, rd, int32_t(rs2));
            return 1;
        case 2: {  // C.LWSP
            if (!rd) return 0;
            uint32_t imm = (bit(c, 12) << 5) | (bits(c, 6, 4) << 2) | (bits(c, 3, 2) << 6);
            *out = enc_i(0x03, rd, 2, 2, int32_t(imm));
            return 1;
        }
        case 4:
            if (!bit(c, 12)) {
                if (rs2 == 0) {  // C.JR
                    if (!rd) return 0;
                    *out = enc_i(0x67, 0, 0, rd, 0);
                } else {  // C.MV
                    *out = enc_r(0, rs2, 0, 0, rd);
                }
            } else {
                if (rd == 0 && rs2 == 0) {  // C.EBREAK
                    *out = 0x00100073;
                } else if (rs2 == 0) {  // C.JALR
                    *out = enc_i(0x67, 1, 0, rd, 0);
                } else {  // C.ADD
                    *out = enc_r(0, rs2, rd, 0, rd);
                }
            }
            return 1;
        case 6: {  // C.SWSP
            uint32_t imm = (bits(c, 12, 9) << 2) | (bits(c, 8, 7) << 6);
            *out = enc_s(0x23, 2, 2, rs2, int32_t(imm));
            return 1;
        }
        }
        return 0;
    }
    return 0;
}

void Rv32Cpu::reset(uint32_t new_pc) {
    for (auto& r : x) r = 0;
    pc = new_pc;
    mstatus = MSTATUS_MPP;
    mie = mip = mtvec = mscratch = mepc = mcause = mtval = 0;
    halted = waiting = false;
    halt_reason.clear();
    lr_valid_ = false;
    attention_ = true;
    core_acc_ = 0;
    core_mark_ = cycles;
    flush_icache();
}

void Rv32Cpu::set_cycle_cost(uint32_t cost) {
    if (cost == 0) cost = 1;
    if (cost == cycle_cost) return;
    core_acc_ = core_cycles();
    core_mark_ = cycles;
    cycle_cost = cost;
}

void Rv32Cpu::set_irq_lines(uint32_t lines) {
    if (lines == mip) return;
    mip = lines;
    attention_ = true;
}

void Rv32Cpu::set_irq(int cause, bool level) {
    attention_ = true;
    if (level) {
        mip |= 1u << cause;
        waiting = false;
    } else {
        mip &= ~(1u << cause);
    }
}

void Rv32Cpu::trap(uint32_t cause, uint32_t tval) {
    attention_ = true;
    if (mtvec == 0 && !(cause & 0x80000000u)) {
        halted = true;
        stop_ = true;
        if (on_fatal_trap) on_fatal_trap(*this, cause, tval);
        return;
    }
    mepc = pc;
    mcause = cause;
    mtval = tval;
    uint32_t ms = mstatus;
    ms = (ms & ~MSTATUS_MPIE) | ((ms & MSTATUS_MIE) ? MSTATUS_MPIE : 0);
    ms &= ~MSTATUS_MIE;
    mstatus = ms | MSTATUS_MPP;
    uint32_t base = mtvec & ~3u;
    if ((cause & 0x80000000u) && (mtvec & 3) == 1)
        pc = base + 4 * (cause & 0x7FFFFFFFu);
    else
        pc = base;
}

bool Rv32Cpu::check_interrupts() {
    uint32_t pending = mip & (mie | mie_bypass);
    if (!pending || !(mstatus & MSTATUS_MIE)) return false;
    int n = 0;
    while (!(pending & (1u << n))) ++n;
    trap(0x80000000u | uint32_t(n), 0);
    return true;
}

bool Rv32Cpu::csr_access(uint32_t csr, uint32_t* old, uint32_t v, bool do_write) {
    uint32_t* reg = nullptr;
    switch (csr) {
    case 0x300: reg = &mstatus; break;
    case 0x301: *old = misa; return true;
    case 0x304: reg = &mie; break;
    case 0x305: reg = &mtvec; break;
    case 0x340: reg = &mscratch; break;
    case 0x341: reg = &mepc; break;
    case 0x342: reg = &mcause; break;
    case 0x343: reg = &mtval; break;
    case 0x344: reg = &mip; break;
    case 0xF11: case 0xF12: case 0xF13: case 0xF14:  // vendor/arch/impl/hart id
        *old = 0;
        return true;
    case 0xB00: case 0xC00: case 0xC01:
        *old = uint32_t(core_cycles());
        if (do_write && csr == 0xB00) core_acc_ += v - uint32_t(core_cycles());
        return true;
    case 0xB80: case 0xC80: case 0xC81:
        *old = uint32_t(core_cycles() >> 32);
        return true;
    case 0xB02: case 0xC02: *old = uint32_t(instret); return true;
    case 0xB82: case 0xC82: *old = uint32_t(instret >> 32); return true;
    default:
        if (custom_csr) {
            uint32_t val = 0;
            if (!custom_csr(csr, &val, false)) return false;
            *old = val;
            if (do_write) custom_csr(csr, &v, true);
            return true;
        }
        return false;
    }
    *old = *reg;
    if (do_write) {
        if (reg == &mstatus) v = (v & (MSTATUS_MIE | MSTATUS_MPIE)) | MSTATUS_MPP;
        *reg = v;
    }
    return true;
}

namespace {
enum Op : uint8_t {
    OP_SLOW, OP_CILL,  // generic decoder (imm = encoding, rd = length); illegal compressed (imm = raw)
    OP_LUI,            // also AUIPC, with the result precomputed
    OP_JAL, OP_JALR,
    OP_BEQ, OP_BNE, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU,
    OP_LB, OP_LH, OP_LW, OP_LBU, OP_LHU, OP_SB, OP_SH, OP_SW,
    OP_ADDI, OP_SLTI, OP_SLTIU, OP_XORI, OP_ORI, OP_ANDI, OP_SLLI, OP_SRLI, OP_SRAI,
    OP_ADD, OP_SUB, OP_SLL, OP_SLT, OP_SLTU, OP_XOR, OP_SRL, OP_SRA, OP_OR, OP_AND,
    OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU, OP_DIV, OP_DIVU, OP_REM, OP_REMU,
    OP_FENCE,
    OP_END,  // end marker after the last instruction of a block that does not end in a jump
};
constexpr uint8_t kSink = 32;  // destination register index for writes to x0
}  // namespace

Rv32Cpu::Rv32Cpu(Bus& bus) : bus_(bus) {
    bus_.on_code_write = [this] {
        flush_icache();
        attention_ = true;  // leave the running block: its code may have changed
    };
}

void Rv32Cpu::decode(uint32_t insn, uint32_t len, uint32_t at, Op& o, bool& ends_block) {
    const uint32_t rd = bits(insn, 11, 7);
    o.rd = uint8_t(rd ? rd : kSink);
    o.rs1 = uint8_t(bits(insn, 19, 15));
    o.rs2 = uint8_t(bits(insn, 24, 20));
    o.imm = 0;
    o.op = OP_SLOW;
    ends_block = false;
    const uint32_t f3 = bits(insn, 14, 12), f7 = insn >> 25;
    switch (insn & 0x7F) {
    case 0x37: o.op = OP_LUI; o.imm = insn & 0xFFFFF000u; break;
    case 0x17: o.op = OP_LUI; o.imm = at + (insn & 0xFFFFF000u); break;  // AUIPC
    case 0x6F:
        o.op = OP_JAL;
        o.imm = at + uint32_t(sext((bit(insn, 31) << 20) | (bits(insn, 19, 12) << 12) | (bit(insn, 20) << 11) |
                                       (bits(insn, 30, 21) << 1),
                                   21));
        ends_block = true;
        break;
    case 0x67: o.op = OP_JALR; o.imm = uint32_t(sext(insn >> 20, 12)); ends_block = true; break;
    case 0x63: {
        static const uint8_t ops[8] = {OP_BEQ, OP_BNE, OP_SLOW, OP_SLOW, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU};
        o.op = ops[f3];
        if (o.op != OP_SLOW)
            o.imm = at + uint32_t(sext((bit(insn, 31) << 12) | (bit(insn, 7) << 11) | (bits(insn, 30, 25) << 5) |
                                           (bits(insn, 11, 8) << 1),
                                       13));
        // Not-taken branches fall through within the block (a side exit).
        break;
    }
    case 0x03: {
        static const uint8_t ops[8] = {OP_LB, OP_LH, OP_LW, OP_SLOW, OP_LBU, OP_LHU, OP_SLOW, OP_SLOW};
        o.op = ops[f3];
        o.imm = uint32_t(sext(insn >> 20, 12));
        break;
    }
    case 0x23: {
        static const uint8_t ops[8] = {OP_SB, OP_SH, OP_SW, OP_SLOW, OP_SLOW, OP_SLOW, OP_SLOW, OP_SLOW};
        o.op = ops[f3];
        o.imm = uint32_t(sext((f7 << 5) | rd, 12));
        break;
    }
    case 0x13: {
        static const uint8_t ops[8] = {OP_ADDI, OP_SLLI, OP_SLTI, OP_SLTIU, OP_XORI, OP_SRLI, OP_ORI, OP_ANDI};
        o.op = ops[f3];
        o.imm = uint32_t(sext(insn >> 20, 12));
        if (f3 == 5 && (f7 & 0x20)) o.op = OP_SRAI;
        if (f3 == 1 || f3 == 5) o.imm = o.rs2;
        break;
    }
    case 0x33:
        if (f7 == 1) {
            static const uint8_t ops[8] = {OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU, OP_DIV, OP_DIVU, OP_REM, OP_REMU};
            o.op = ops[f3];
        } else if (f7 == 0) {
            static const uint8_t ops[8] = {OP_ADD, OP_SLL, OP_SLT, OP_SLTU, OP_XOR, OP_SRL, OP_OR, OP_AND};
            o.op = ops[f3];
        } else if (f7 == 0x20 && (f3 == 0 || f3 == 5)) {
            o.op = f3 == 0 ? OP_SUB : OP_SRA;
        }
        break;
    case 0x0F: o.op = OP_FENCE; break;
    default: break;  // SYSTEM, AMO, illegal: slow path
    }
    if (o.op == OP_SLOW) {
        o.imm = insn;
        o.rd = uint8_t(len);
        ends_block = true;  // CSR writes, traps, WFI and AMOs need the full machine state
    }
}

void Rv32Cpu::flush_icache() {
    for (uint32_t i = 0; i < (1u << kBlockBits); ++i) blocks_[i].pc = 1;
    arena_used_ = 0;
    bus_.clear_code_marks();
}

const Rv32Cpu::Block* Rv32Cpu::translate(uint32_t start, uint32_t max_ops) {
    Block* b = &partial_;
    Op* dst = partial_ops_.get();
    if (max_ops >= kMaxBlockOps) {
        if (arena_used_ + kMaxBlockOps + 1 > kArenaOps) flush_icache();
        b = &blocks_[(start >> 1) & ((1u << kBlockBits) - 1)];
        dst = arena_.get() + arena_used_;
        max_ops = kMaxBlockOps;
    }
    uint32_t at = start, count = 0;
    for (;;) {
        Op& o = dst[count];
        uint32_t raw = bus_.read<2>(at);
        const bool is32 = (raw & 3) == 3;
        if (is32) raw |= bus_.read<2>(at + 2) << 16;
        uint32_t insn = raw;
        bool ends = true;
        if (!is32 && !expand_compressed(uint16_t(raw), &insn)) {
            o.op = OP_CILL;
            o.rd = o.rs1 = o.rs2 = 0;
            o.imm = raw;
        } else {
            decode(insn, is32 ? 4 : 2, at, o, ends);
        }
        o.pc = at;
        ++count;
        at += is32 ? 4 : 2;
        if (ends || count == max_ops || at - hook_lo < hook_hi - hook_lo) break;
    }
    dst[count] = Op{OP_END, 0, 0, 0, 0, at};
    b->pc = start;
    b->count = count;
    b->ops = dst;
    if (b != &partial_) arena_used_ += count + 1;
    bus_.mark_code(start, at - start);
    return b;
}

// GCC and Clang dispatch through a table of label addresses ("threaded code"),
// which gives every handler its own indirect jump and so far better branch
// prediction than one shared switch. Other compilers use the switch.
#if defined(__GNUC__) && !defined(RV_NO_THREADED)
#define RV_THREADED 1
#endif

uint64_t Rv32Cpu::run(uint64_t max_instructions) {
    stop_ = false;
    attention_ = true;
    uint64_t n = 0;
    uint32_t* const r = x;
    const uint32_t mask = (1u << kBlockBits) - 1;
    const bool hooked = hook && hook_hi > hook_lo;
#ifdef RV_THREADED
    static const void* const kHandlers[] = {
        &&L_OP_SLOW, &&L_OP_CILL, &&L_OP_LUI, &&L_OP_JAL, &&L_OP_JALR,
        &&L_OP_BEQ, &&L_OP_BNE, &&L_OP_BLT, &&L_OP_BGE, &&L_OP_BLTU, &&L_OP_BGEU,
        &&L_OP_LB, &&L_OP_LH, &&L_OP_LW, &&L_OP_LBU, &&L_OP_LHU, &&L_OP_SB, &&L_OP_SH, &&L_OP_SW,
        &&L_OP_ADDI, &&L_OP_SLTI, &&L_OP_SLTIU, &&L_OP_XORI, &&L_OP_ORI, &&L_OP_ANDI, &&L_OP_SLLI, &&L_OP_SRLI,
        &&L_OP_SRAI, &&L_OP_ADD, &&L_OP_SUB, &&L_OP_SLL, &&L_OP_SLT, &&L_OP_SLTU, &&L_OP_XOR, &&L_OP_SRL,
        &&L_OP_SRA, &&L_OP_OR, &&L_OP_AND, &&L_OP_MUL, &&L_OP_MULH, &&L_OP_MULHSU, &&L_OP_MULHU, &&L_OP_DIV,
        &&L_OP_DIVU, &&L_OP_REM, &&L_OP_REMU, &&L_OP_FENCE, &&L_OP_END,
    };
    static_assert(sizeof(kHandlers) / sizeof(kHandlers[0]) == OP_END + 1, "handler table out of sync");
#define OPCASE(name) L_##name:
#define NEXT()                     \
    do {                           \
        ++o;                       \
        goto *kHandlers[o->op];    \
    } while (0)
#else
#define OPCASE(name) case name:
#define NEXT()                     \
    do {                           \
        ++o;                       \
        goto dispatch;             \
    } while (0)
#endif
#define A r[o->rs1]
#define C r[o->rs2]
#define D r[o->rd]

    while (n < max_instructions) {
        if (attention_) {
            if (halted || stop_) break;
            if (mip && check_interrupts()) waiting = false;
            if (waiting) {
                cycles += (max_instructions - n) * cycle_cost;  // sleeping in WFI: time still passes
                break;
            }
            attention_ = (mip & (mie | mie_bypass)) != 0;
        }
        if (hooked && pc - hook_lo < hook_hi - hook_lo) {
            if (hook(*this)) {
                ++n;
                ++instret;
                cycles += cycle_cost;
                continue;
            }
            if (halted) break;
        }

        // Time and pc are only written back at the end of the block, or when
        // code outside it may look at them (sync).
        const Block* b = &blocks_[(pc >> 1) & mask];
        if (b->pc != pc) b = translate(pc, kMaxBlockOps);
        if (b->count > max_instructions - n) b = translate(pc, uint32_t(max_instructions - n));
        const Op* const ops = b->ops;
        uint32_t next = 0;
        const uint64_t base = cycles;
        const uint32_t cost = cycle_cost;
        const Op* o = ops;
        uint32_t done;
        auto sync = [&] {
            pc = o->pc;
            cycles = base + uint64_t(o - ops) * cost;
        };

        // A load from MMIO may have raised an interrupt or a stop: leave the
        // block after it.
#define LOAD(N, T)                                                         \
    {                                                                      \
        const uint32_t ea = A + o->imm;                                    \
        const Bus::Page& p = bus_.page(ea);                                \
        const uint32_t off = ea & Bus::kPageMask;                          \
        if (p.ptr && off <= Bus::kPageSize - N) {                          \
            T t;                                                           \
            std::memcpy(&t, p.ptr + off, N);                               \
            D = uint32_t(t);                                               \
            NEXT();                                                        \
        }                                                                  \
        sync();                                                            \
        D = uint32_t(T(bus_.read<N>(ea)));                                 \
        if (!attention_) NEXT();                                           \
        next = o[1].pc;                                                    \
        ++o;                                                               \
        goto out;                                                          \
    }
        // MMIO and stores into translated code take the bus slow path and
        // then return to the run loop, which picks up interrupts, clock
        // changes and new code.
#define STORE(N)                                                           \
    {                                                                      \
        const uint32_t ea = A + o->imm, v = C;                             \
        const Bus::Page& p = bus_.page(ea);                                \
        const uint32_t off = ea & Bus::kPageMask;                          \
        lr_valid_ = false;                                                 \
        if (p.ptr && off <= Bus::kPageSize - N) {                          \
            if (!p.writable) NEXT(); /* ROM, flash cache: ignored */       \
            if (!(p.code[off >> 2] | p.code[(off + N - 1) >> 2])) {        \
                std::memcpy(p.ptr + off, &v, N);                           \
                NEXT();                                                    \
            }                                                              \
        }                                                                  \
        sync();                                                            \
        bus_.write<N>(ea, v);                                              \
        pc = o[1].pc;                                                      \
        goto out_synced;                                                   \
    }
#define BRANCH(cond)                                                       \
    {                                                                      \
        if (cond) {                                                        \
            next = o->imm;                                                 \
            ++o;                                                           \
            goto out;                                                      \
        }                                                                  \
        NEXT();                                                            \
    }

#ifdef RV_THREADED
        goto *kHandlers[o->op];
#else
    dispatch:
        switch (o->op) {
#endif
        OPCASE(OP_LUI) { D = o->imm; NEXT(); }
        OPCASE(OP_JAL) { D = o[1].pc; next = o->imm; ++o; goto out; }
        OPCASE(OP_JALR) { next = (A + o->imm) & ~1u; D = o[1].pc; ++o; goto out; }
        OPCASE(OP_BEQ) BRANCH(A == C)
        OPCASE(OP_BNE) BRANCH(A != C)
        OPCASE(OP_BLT) BRANCH(int32_t(A) < int32_t(C))
        OPCASE(OP_BGE) BRANCH(int32_t(A) >= int32_t(C))
        OPCASE(OP_BLTU) BRANCH(A < C)
        OPCASE(OP_BGEU) BRANCH(A >= C)
        OPCASE(OP_LB) LOAD(1, int8_t)
        OPCASE(OP_LH) LOAD(2, int16_t)
        OPCASE(OP_LW) LOAD(4, uint32_t)
        OPCASE(OP_LBU) LOAD(1, uint8_t)
        OPCASE(OP_LHU) LOAD(2, uint16_t)
        OPCASE(OP_SB) STORE(1)
        OPCASE(OP_SH) STORE(2)
        OPCASE(OP_SW) STORE(4)
        OPCASE(OP_ADDI) { D = A + o->imm; NEXT(); }
        OPCASE(OP_SLTI) { D = int32_t(A) < int32_t(o->imm); NEXT(); }
        OPCASE(OP_SLTIU) { D = A < o->imm; NEXT(); }
        OPCASE(OP_XORI) { D = A ^ o->imm; NEXT(); }
        OPCASE(OP_ORI) { D = A | o->imm; NEXT(); }
        OPCASE(OP_ANDI) { D = A & o->imm; NEXT(); }
        OPCASE(OP_SLLI) { D = A << o->imm; NEXT(); }
        OPCASE(OP_SRLI) { D = A >> o->imm; NEXT(); }
        OPCASE(OP_SRAI) { D = uint32_t(int32_t(A) >> o->imm); NEXT(); }
        OPCASE(OP_ADD) { D = A + C; NEXT(); }
        OPCASE(OP_SUB) { D = A - C; NEXT(); }
        OPCASE(OP_SLL) { D = A << (C & 31); NEXT(); }
        OPCASE(OP_SLT) { D = int32_t(A) < int32_t(C); NEXT(); }
        OPCASE(OP_SLTU) { D = A < C; NEXT(); }
        OPCASE(OP_XOR) { D = A ^ C; NEXT(); }
        OPCASE(OP_SRL) { D = A >> (C & 31); NEXT(); }
        OPCASE(OP_SRA) { D = uint32_t(int32_t(A) >> (C & 31)); NEXT(); }
        OPCASE(OP_OR) { D = A | C; NEXT(); }
        OPCASE(OP_AND) { D = A & C; NEXT(); }
        OPCASE(OP_MUL) { D = A * C; NEXT(); }
        OPCASE(OP_MULH) { D = uint32_t((int64_t(int32_t(A)) * int64_t(int32_t(C))) >> 32); NEXT(); }
        OPCASE(OP_MULHSU) { D = uint32_t((int64_t(int32_t(A)) * int64_t(uint64_t(C))) >> 32); NEXT(); }
        OPCASE(OP_MULHU) { D = uint32_t((uint64_t(A) * uint64_t(C)) >> 32); NEXT(); }
        OPCASE(OP_DIV) {
            const uint32_t a = A, c = C;
            D = c == 0 ? 0xFFFFFFFFu : (a == 0x80000000u && c == 0xFFFFFFFFu) ? a : uint32_t(int32_t(a) / int32_t(c));
            NEXT();
        }
        OPCASE(OP_DIVU) { const uint32_t a = A, c = C; D = c ? a / c : 0xFFFFFFFFu; NEXT(); }
        OPCASE(OP_REM) {
            const uint32_t a = A, c = C;
            D = c == 0 ? a : (a == 0x80000000u && c == 0xFFFFFFFFu) ? 0 : uint32_t(int32_t(a) % int32_t(c));
            NEXT();
        }
        OPCASE(OP_REMU) { const uint32_t a = A, c = C; D = c ? a % c : a; NEXT(); }
        OPCASE(OP_FENCE) { NEXT(); }
        OPCASE(OP_END) { next = o->pc; goto out; }
        OPCASE(OP_SLOW)
        OPCASE(OP_CILL) {  // always the last op of a block
            sync();
            done = uint32_t(o - ops);
            instret += done;  // minstret reads see the instructions before this one
            n += done;
            if (o->op == OP_CILL) trap(kIllegal, o->imm);
            else execute(o->imm, o->rd);
            attention_ = true;  // CSR writes, traps, WFI, AMOs
            ++o;
            goto out_synced_counted;
        }
#ifndef RV_THREADED
        }
#endif
    out:
        done = uint32_t(o - ops);
        pc = next;
        cycles = base + uint64_t(done) * cost;
        instret += done;
        n += done;
        continue;
    out_synced:  // pc and cycles are exact up to the start of instruction o
        done = uint32_t(o - ops) + 1;
        instret += done;
        n += done;
        cycles += cycle_cost;
        continue;
    out_synced_counted:  // the same after a slow-path instruction, already counted
        ++instret;
        ++n;
        cycles += cycle_cost;
    }
    return n;
#undef LOAD
#undef STORE
#undef BRANCH
#undef OPCASE
#undef NEXT
#undef A
#undef C
#undef D
}

void Rv32Cpu::execute(uint32_t insn, uint32_t len) {
    const uint32_t opcode = insn & 0x7F;
    const uint32_t rd = bits(insn, 11, 7);
    const uint32_t f3 = bits(insn, 14, 12);
    const uint32_t rs1 = bits(insn, 19, 15);
    const uint32_t rs2 = bits(insn, 24, 20);
    const uint32_t f7 = insn >> 25;
    const uint32_t a = x[rs1], b = x[rs2];
    uint32_t next = pc + len;
    uint32_t result = 0;
    bool write_rd = true;

    switch (opcode) {
    case 0x37: result = insn & 0xFFFFF000u; break;  // LUI
    case 0x17: result = pc + (insn & 0xFFFFF000u); break;  // AUIPC
    case 0x6F: {  // JAL
        int32_t imm = sext((bit(insn, 31) << 20) | (bits(insn, 19, 12) << 12) | (bit(insn, 20) << 11) |
                               (bits(insn, 30, 21) << 1),
                           21);
        result = next;
        next = pc + uint32_t(imm);
        break;
    }
    case 0x67:  // JALR
        result = next;
        next = (a + uint32_t(sext(insn >> 20, 12))) & ~1u;
        break;
    case 0x63: {  // BRANCH
        write_rd = false;
        bool take;
        switch (f3) {
        case 0: take = a == b; break;
        case 1: take = a != b; break;
        case 4: take = int32_t(a) < int32_t(b); break;
        case 5: take = int32_t(a) >= int32_t(b); break;
        case 6: take = a < b; break;
        case 7: take = a >= b; break;
        default: trap(kIllegal, insn); return;
        }
        if (take) {
            int32_t imm = sext((bit(insn, 31) << 12) | (bit(insn, 7) << 11) | (bits(insn, 30, 25) << 5) |
                                   (bits(insn, 11, 8) << 1),
                               13);
            next = pc + uint32_t(imm);
        }
        break;
    }
    case 0x03: {  // LOAD
        uint32_t addr = a + uint32_t(sext(insn >> 20, 12));
        switch (f3) {
        case 0: result = uint32_t(sext(bus_.read<1>(addr), 8)); break;
        case 1: result = uint32_t(sext(bus_.read<2>(addr), 16)); break;
        case 2: result = bus_.read<4>(addr); break;
        case 4: result = bus_.read<1>(addr); break;
        case 5: result = bus_.read<2>(addr); break;
        default: trap(kIllegal, insn); return;
        }
        break;
    }
    case 0x23: {  // STORE
        write_rd = false;
        uint32_t addr = a + uint32_t(sext((f7 << 5) | rd, 12));
        switch (f3) {
        case 0: bus_.write<1>(addr, b); break;
        case 1: bus_.write<2>(addr, b); break;
        case 2: bus_.write<4>(addr, b); break;
        default: trap(kIllegal, insn); return;
        }
        lr_valid_ = false;
        break;
    }
    case 0x13: {  // OP-IMM
        uint32_t imm = uint32_t(sext(insn >> 20, 12));
        uint32_t sh = rs2;
        switch (f3) {
        case 0: result = a + imm; break;
        case 1: result = a << sh; break;
        case 2: result = int32_t(a) < int32_t(imm); break;
        case 3: result = a < imm; break;
        case 4: result = a ^ imm; break;
        case 5: result = (f7 & 0x20) ? uint32_t(int32_t(a) >> sh) : a >> sh; break;
        case 6: result = a | imm; break;
        case 7: result = a & imm; break;
        }
        break;
    }
    case 0x33:  // OP
        if (f7 == 1) {  // M extension
            switch (f3) {
            case 0: result = a * b; break;
            case 1: result = uint32_t((int64_t(int32_t(a)) * int64_t(int32_t(b))) >> 32); break;
            case 2: result = uint32_t((int64_t(int32_t(a)) * int64_t(uint64_t(b))) >> 32); break;
            case 3: result = uint32_t((uint64_t(a) * uint64_t(b)) >> 32); break;
            case 4:
                if (b == 0) result = 0xFFFFFFFFu;
                else if (a == 0x80000000u && b == 0xFFFFFFFFu) result = a;
                else result = uint32_t(int32_t(a) / int32_t(b));
                break;
            case 5: result = b ? a / b : 0xFFFFFFFFu; break;
            case 6:
                if (b == 0) result = a;
                else if (a == 0x80000000u && b == 0xFFFFFFFFu) result = 0;
                else result = uint32_t(int32_t(a) % int32_t(b));
                break;
            case 7: result = b ? a % b : a; break;
            }
        } else {
            switch (f3) {
            case 0: result = (f7 & 0x20) ? a - b : a + b; break;
            case 1: result = a << (b & 31); break;
            case 2: result = int32_t(a) < int32_t(b); break;
            case 3: result = a < b; break;
            case 4: result = a ^ b; break;
            case 5: result = (f7 & 0x20) ? uint32_t(int32_t(a) >> (b & 31)) : a >> (b & 31); break;
            case 6: result = a | b; break;
            case 7: result = a & b; break;
            }
        }
        break;
    case 0x0F:  // FENCE / FENCE.I
        write_rd = false;
        break;
    case 0x2F: {  // AMO (RV32A)
        if (f3 != 2) { trap(kIllegal, insn); return; }
        uint32_t f5 = f7 >> 2;
        if (a & 3) { trap(f5 == 2 ? kLoadMisaligned : kStoreMisaligned, a); return; }
        if (f5 == 2) {  // LR.W
            result = bus_.read<4>(a);
            lr_addr_ = a;
            lr_valid_ = true;
        } else if (f5 == 3) {  // SC.W
            if (lr_valid_ && lr_addr_ == a) {
                bus_.write<4>(a, b);
                result = 0;
            } else {
                result = 1;
            }
            lr_valid_ = false;
        } else {
            uint32_t old = bus_.read<4>(a), nv;
            switch (f5) {
            case 0x01: nv = b; break;                // AMOSWAP
            case 0x00: nv = old + b; break;          // AMOADD
            case 0x04: nv = old ^ b; break;          // AMOXOR
            case 0x0C: nv = old & b; break;          // AMOAND
            case 0x08: nv = old | b; break;          // AMOOR
            case 0x10: nv = int32_t(old) < int32_t(b) ? old : b; break;  // AMOMIN
            case 0x14: nv = int32_t(old) > int32_t(b) ? old : b; break;  // AMOMAX
            case 0x18: nv = old < b ? old : b; break;  // AMOMINU
            case 0x1C: nv = old > b ? old : b; break;  // AMOMAXU
            default: trap(kIllegal, insn); return;
            }
            bus_.write<4>(a, nv);
            result = old;
        }
        break;
    }
    case 0x73: {  // SYSTEM
        if (f3 == 0) {
            write_rd = false;
            uint32_t f12 = insn >> 20;
            if (insn == 0x00000073) { trap(kEcallM, 0); return; }
            if (insn == 0x00100073) { trap(kBreakpoint, pc); return; }
            if (insn == 0x30200073) {  // MRET
                uint32_t ms = mstatus;
                ms = (ms & ~MSTATUS_MIE) | ((ms & MSTATUS_MPIE) ? MSTATUS_MIE : 0);
                mstatus = ms | MSTATUS_MPIE | MSTATUS_MPP;
                pc = mepc;
                return;
            }
            if (insn == 0x10500073) {  // WFI
                if (!(mip & (mie | mie_bypass))) waiting = true;
                break;
            }
            (void)f12;
            trap(kIllegal, insn);
            return;
        }
        uint32_t csr = insn >> 20;
        uint32_t src = (f3 & 4) ? rs1 : a;  // immediate forms use rs1 field as zimm
        uint32_t old = 0;
        bool do_write;
        uint32_t nv;
        switch (f3 & 3) {
        case 1: do_write = true; nv = src; break;                  // CSRRW
        case 2: do_write = rs1 != 0; nv = 0; break;                // CSRRS
        default: do_write = rs1 != 0; nv = 0; break;               // CSRRC
        }
        if (!csr_access(csr, &old, 0, false)) { trap(kIllegal, insn); return; }
        if ((f3 & 3) == 2) nv = old | src;
        if ((f3 & 3) == 3) nv = old & ~src;
        if (do_write) csr_access(csr, &old, nv, true);
        result = old;
        break;
    }
    default:
        trap(kIllegal, insn);
        return;
    }
    if (write_rd && rd) x[rd] = result;
    pc = next;
}

}  // namespace esp32sim
