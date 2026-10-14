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
enum Op : uint16_t {
    OP_SLOW = 0, OP_ILLEGAL,
    OP_LUI, OP_AUIPC, OP_JAL, OP_JALR,
    OP_BEQ, OP_BNE, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU,
    OP_LB, OP_LH, OP_LW, OP_LBU, OP_LHU, OP_SB, OP_SH, OP_SW,
    OP_ADDI, OP_SLTI, OP_SLTIU, OP_XORI, OP_ORI, OP_ANDI, OP_SLLI, OP_SRLI, OP_SRAI,
    OP_ADD, OP_SUB, OP_SLL, OP_SLT, OP_SLTU, OP_XOR, OP_SRL, OP_SRA, OP_OR, OP_AND,
    OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU, OP_DIV, OP_DIVU, OP_REM, OP_REMU,
    OP_FENCE,
};
}  // namespace

void Rv32Cpu::decode(uint32_t insn, uint32_t len, uint32_t raw, Decoded& d) {
    d.pc = pc;
    d.raw = raw;
    d.len = uint8_t(len);
    d.insn = insn;
    d.rd = uint8_t(bits(insn, 11, 7));
    d.rs1 = uint8_t(bits(insn, 19, 15));
    d.rs2 = uint8_t(bits(insn, 24, 20));
    d.imm = 0;
    d.op = OP_SLOW;
    const uint32_t f3 = bits(insn, 14, 12), f7 = insn >> 25;
    switch (insn & 0x7F) {
    case 0x37: d.op = OP_LUI; d.imm = int32_t(insn & 0xFFFFF000u); break;
    case 0x17: d.op = OP_AUIPC; d.imm = int32_t(insn & 0xFFFFF000u); break;
    case 0x6F:
        d.op = OP_JAL;
        d.imm = sext((bit(insn, 31) << 20) | (bits(insn, 19, 12) << 12) | (bit(insn, 20) << 11) | (bits(insn, 30, 21) << 1), 21);
        break;
    case 0x67: d.op = OP_JALR; d.imm = sext(insn >> 20, 12); break;
    case 0x63: {
        static const uint16_t ops[8] = {OP_BEQ, OP_BNE, OP_SLOW, OP_SLOW, OP_BLT, OP_BGE, OP_BLTU, OP_BGEU};
        d.op = ops[f3];
        d.imm = sext((bit(insn, 31) << 12) | (bit(insn, 7) << 11) | (bits(insn, 30, 25) << 5) | (bits(insn, 11, 8) << 1), 13);
        break;
    }
    case 0x03: {
        static const uint16_t ops[8] = {OP_LB, OP_LH, OP_LW, OP_SLOW, OP_LBU, OP_LHU, OP_SLOW, OP_SLOW};
        d.op = ops[f3];
        d.imm = sext(insn >> 20, 12);
        break;
    }
    case 0x23: {
        static const uint16_t ops[8] = {OP_SB, OP_SH, OP_SW, OP_SLOW, OP_SLOW, OP_SLOW, OP_SLOW, OP_SLOW};
        d.op = ops[f3];
        d.imm = sext((f7 << 5) | d.rd, 12);
        break;
    }
    case 0x13: {
        static const uint16_t ops[8] = {OP_ADDI, OP_SLLI, OP_SLTI, OP_SLTIU, OP_XORI, OP_SRLI, OP_ORI, OP_ANDI};
        d.op = ops[f3];
        d.imm = sext(insn >> 20, 12);
        if (f3 == 5 && (f7 & 0x20)) d.op = OP_SRAI;
        if (f3 == 1 || f3 == 5) d.imm = int32_t(d.rs2);
        break;
    }
    case 0x33:
        if (f7 == 1) {
            static const uint16_t ops[8] = {OP_MUL, OP_MULH, OP_MULHSU, OP_MULHU, OP_DIV, OP_DIVU, OP_REM, OP_REMU};
            d.op = ops[f3];
        } else if (f7 == 0) {
            static const uint16_t ops[8] = {OP_ADD, OP_SLL, OP_SLT, OP_SLTU, OP_XOR, OP_SRL, OP_OR, OP_AND};
            d.op = ops[f3];
        } else if (f7 == 0x20 && (f3 == 0 || f3 == 5)) {
            d.op = f3 == 0 ? OP_SUB : OP_SRA;
        }
        break;
    case 0x0F: d.op = OP_FENCE; break;
    default: break;  // SYSTEM, AMO, illegal: slow path
    }
}

void Rv32Cpu::flush_icache() {
    for (uint32_t i = 0; i < (1u << kCacheBits); ++i) icache_[i].pc = 1;
}

uint64_t Rv32Cpu::run(uint64_t max_instructions) {
    stop_ = false;
    attention_ = true;
    uint64_t n = 0;
    const uint32_t mask = (1u << kCacheBits) - 1;
    Decoded* cache = icache_.get();
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
        if (pc - hook_lo < hook_hi - hook_lo && hook) {
            if (hook(*this)) {
                ++n;
                cycles += cycle_cost;
                continue;
            }
            if (halted) break;
        }
        Decoded& d = cache[(pc >> 1) & mask];
        if (d.pc != pc) {
            uint32_t raw = bus_.read<2>(pc);
            const bool is32 = (raw & 3) == 3;
            if (is32) raw |= bus_.read<2>(pc + 2) << 16;
            uint32_t insn = raw;
            if (!is32 && !expand_compressed(uint16_t(raw), &insn)) {
                trap(kIllegal, raw);
                attention_ = true;
                ++n;
                cycles += cycle_cost;
                continue;
            }
            decode(insn, is32 ? 4 : 2, raw, d);
        }
        exec(d);
        ++n;
        cycles += cycle_cost;
    }
    instret += n;
    return n;
}

void Rv32Cpu::exec(const Decoded& d) {
    const uint32_t a = x[d.rs1], b = x[d.rs2];
    const uint32_t imm = uint32_t(d.imm);
    uint32_t next = pc + d.len;
    uint32_t r;
    switch (d.op) {
    case OP_LUI: r = imm; break;
    case OP_AUIPC: r = pc + imm; break;
    case OP_JAL: r = next; next = pc + imm; break;
    case OP_JALR: r = next; next = (a + imm) & ~1u; break;
    case OP_BEQ: if (a == b) next = pc + imm; pc = next; return;
    case OP_BNE: if (a != b) next = pc + imm; pc = next; return;
    case OP_BLT: if (int32_t(a) < int32_t(b)) next = pc + imm; pc = next; return;
    case OP_BGE: if (int32_t(a) >= int32_t(b)) next = pc + imm; pc = next; return;
    case OP_BLTU: if (a < b) next = pc + imm; pc = next; return;
    case OP_BGEU: if (a >= b) next = pc + imm; pc = next; return;
    case OP_LB: r = uint32_t(sext(bus_.read<1>(a + imm), 8)); break;
    case OP_LH: r = uint32_t(sext(bus_.read<2>(a + imm), 16)); break;
    case OP_LW: r = bus_.read<4>(a + imm); break;
    case OP_LBU: r = bus_.read<1>(a + imm); break;
    case OP_LHU: r = bus_.read<2>(a + imm); break;
    case OP_SB: bus_.write<1>(a + imm, b); invalidate(a + imm); lr_valid_ = false; pc = next; return;
    case OP_SH: bus_.write<2>(a + imm, b); invalidate(a + imm); lr_valid_ = false; pc = next; return;
    case OP_SW: bus_.write<4>(a + imm, b); invalidate(a + imm); invalidate(a + imm + 2); lr_valid_ = false; pc = next; return;
    case OP_ADDI: r = a + imm; break;
    case OP_SLTI: r = int32_t(a) < int32_t(imm); break;
    case OP_SLTIU: r = a < imm; break;
    case OP_XORI: r = a ^ imm; break;
    case OP_ORI: r = a | imm; break;
    case OP_ANDI: r = a & imm; break;
    case OP_SLLI: r = a << imm; break;
    case OP_SRLI: r = a >> imm; break;
    case OP_SRAI: r = uint32_t(int32_t(a) >> imm); break;
    case OP_ADD: r = a + b; break;
    case OP_SUB: r = a - b; break;
    case OP_SLL: r = a << (b & 31); break;
    case OP_SLT: r = int32_t(a) < int32_t(b); break;
    case OP_SLTU: r = a < b; break;
    case OP_XOR: r = a ^ b; break;
    case OP_SRL: r = a >> (b & 31); break;
    case OP_SRA: r = uint32_t(int32_t(a) >> (b & 31)); break;
    case OP_OR: r = a | b; break;
    case OP_AND: r = a & b; break;
    case OP_MUL: r = a * b; break;
    case OP_MULH: r = uint32_t((int64_t(int32_t(a)) * int64_t(int32_t(b))) >> 32); break;
    case OP_MULHSU: r = uint32_t((int64_t(int32_t(a)) * int64_t(uint64_t(b))) >> 32); break;
    case OP_MULHU: r = uint32_t((uint64_t(a) * uint64_t(b)) >> 32); break;
    case OP_DIV:
        r = b == 0 ? 0xFFFFFFFFu : (a == 0x80000000u && b == 0xFFFFFFFFu) ? a : uint32_t(int32_t(a) / int32_t(b));
        break;
    case OP_DIVU: r = b ? a / b : 0xFFFFFFFFu; break;
    case OP_REM:
        r = b == 0 ? a : (a == 0x80000000u && b == 0xFFFFFFFFu) ? 0 : uint32_t(int32_t(a) % int32_t(b));
        break;
    case OP_REMU: r = b ? a % b : a; break;
    case OP_FENCE: pc = next; return;
    default:
        execute(d.insn, d.len);
        attention_ = true;  // CSR writes, traps, WFI, AMOs
        return;
    }
    if (d.rd) x[d.rd] = r;
    pc = next;
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
        invalidate(addr);
        invalidate(addr + 2);
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
                invalidate(a);
                invalidate(a + 2);
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
            invalidate(a);
            invalidate(a + 2);
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
