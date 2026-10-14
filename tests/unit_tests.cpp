// Unit tests for the CPU core and utilities. Run: ctest or ./esp32sim_tests
#include <cstdio>
#include <cstring>
#include <vector>

#include "core/rv32.hpp"
#include "soc/esp32c3/crypto.hpp"
#include "soc/esp32c3/intmatrix.hpp"
#include "soc/esp32c6/peripherals.hpp"
#include "util/inflate.hpp"
#include "util/md5.hpp"

using namespace esp32sim;

static int g_failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

class RamBus : public Bus {
public:
    RamBus() : mem(0x10000) { map(0x40000000, mem.data(), 0x10000, true); }
    uint32_t mmio_read(uint32_t, int) override { return 0; }
    void mmio_write(uint32_t, uint32_t, int) override {}
    std::vector<uint8_t> mem;
    void put32(uint32_t addr, uint32_t v) { std::memcpy(&mem[addr - 0x40000000], &v, 4); }
    void put16(uint32_t addr, uint16_t v) { std::memcpy(&mem[addr - 0x40000000], &v, 2); }
};

static void test_base_isa() {
    RamBus bus;
    Rv32Cpu cpu(bus);
    const uint32_t prog[] = {
        0x00500093,  // addi x1, x0, 5
        0x00700113,  // addi x2, x0, 7
        0x022081b3,  // mul  x3, x1, x2      = 35
        0x40118233,  // sub  x4, x3, x1      = 30
        0x0220c2b3,  // div  x5, x1, x2      = 0
        0x0221e333,  // rem  x6, x3, x2      = 0
        0xfff00393,  // addi x7, x0, -1
        0x0013d413,  // srli x8, x7, 1       = 0x7fffffff
        0x4013d493,  // srai x9, x7, 1       = -1
        0x123452b7,  // lui  x5, 0x12345
        0x00001517,  // auipc x10, 1
        0x40000637,  // lui x12, 0x40000
        0x10562023,  // sw x5, 0x100(x12)
        0x10064683,  // lbu x13, 0x100(x12)  = 0x00 (low byte of 0x12345000)
        0x10162703,  // lw   x14, 0x101(x12)  misaligned = 0x00123450
        0x00000073,  // ecall -> fatal trap stops
    };
    for (size_t i = 0; i < sizeof(prog) / 4; ++i) bus.put32(0x40000000 + 4 * uint32_t(i), prog[i]);
    bool trapped = false;
    cpu.on_fatal_trap = [&](Rv32Cpu&, uint32_t cause, uint32_t) { trapped = cause == 11; };
    cpu.reset(0x40000000);
    cpu.run(100);
    CHECK(trapped);
    CHECK(cpu.x[3] == 35);
    CHECK(cpu.x[4] == 30);
    CHECK(cpu.x[6] == 0);
    CHECK(cpu.x[8] == 0x7fffffff);
    CHECK(cpu.x[9] == 0xffffffff);
    CHECK(cpu.x[5] == 0x12345000);
    CHECK(cpu.x[10] == 0x40000028 + 0x1000);
    CHECK(cpu.x[13] == 0x00);
    CHECK(cpu.x[14] == 0x00123450);
}

static void test_compressed_loop() {
    // Sum 1..10 using compressed instructions:
    //   c.li a0,0 ; c.li a1,10 ; loop: c.add a0,a1 ; c.addi a1,-1 ; c.bnez a1,loop ; ebreak
    RamBus bus;
    Rv32Cpu cpu(bus);
    const uint16_t prog[] = {0x4501, 0x45a9, 0x952e, 0x15fd, 0xfdf5, 0x9002};
    for (size_t i = 0; i < 6; ++i) bus.put16(0x40000000 + 2 * uint32_t(i), prog[i]);
    bool brk = false;
    cpu.on_fatal_trap = [&](Rv32Cpu&, uint32_t cause, uint32_t) { brk = cause == 3; };
    cpu.reset(0x40000000);
    cpu.run(1000);
    CHECK(brk);
    CHECK(cpu.x[10] == 55);
}

static void test_trap_and_mret() {
    RamBus bus;
    Rv32Cpu cpu(bus);
    // handler at 0x40000100: csrr t0, mepc ; addi t0,t0,4 ; csrw mepc,t0 ; addi s1,s1,1 ; mret
    const uint32_t handler[] = {0x341022f3, 0x00428293, 0x34129073, 0x00148493, 0x30200073};
    for (size_t i = 0; i < 5; ++i) bus.put32(0x40000100 + 4 * uint32_t(i), handler[i]);
    // main: la t1, handler ; csrw mtvec, t1 ; ecall ; ecall ; ebreak-free spin: addi s0,s0,1 ; j .
    const uint32_t mainp[] = {0x40000337, 0x10030313, 0x30531073, 0x00000073, 0x00000073, 0x00140413, 0x0000006f};
    for (size_t i = 0; i < 7; ++i) bus.put32(0x40000000 + 4 * uint32_t(i), mainp[i]);
    cpu.reset(0x40000000);
    cpu.run(50);
    CHECK(cpu.x[9] == 2);
    CHECK(cpu.x[8] == 1);
    CHECK(cpu.mcause == 11);
}

// Code patched through a second address of the same RAM (like the C3's data
// bus alias of IRAM) must run in its new form the next time round.
static void test_self_modifying_code() {
    RamBus bus;
    bus.map(0x50000000, bus.mem.data(), 0x10000, true);
    Rv32Cpu cpu(bus);
    const uint32_t prog[] = {
        0x00100513,  // loop: addi a0, x0, 1   (patched to addi a0, x0, 2)
        0x00a40433,  //       add  s0, s0, a0
        0x0062a023,  //       sw   t1, 0(t0)
        0x00148493,  //       addi s1, s1, 1
        0x00200393,  //       addi t2, x0, 2
        0xfe74c6e3,  //       blt  s1, t2, loop
        0x00100073,  //       ebreak
    };
    for (size_t i = 0; i < sizeof(prog) / 4; ++i) bus.put32(0x40000000 + 4 * uint32_t(i), prog[i]);
    bool brk = false;
    cpu.on_fatal_trap = [&](Rv32Cpu&, uint32_t cause, uint32_t) { brk = cause == 3; };
    cpu.reset(0x40000000);
    cpu.x[5] = 0x50000000;  // t0: the patched instruction, through the alias
    cpu.x[6] = 0x00200513;  // t1: addi a0, x0, 2
    cpu.run(100);
    CHECK(brk);
    CHECK(cpu.x[8] == 1 + 2);
}

// run(n) executes exactly n instructions, and time and minstret follow.
static void test_run_budget() {
    RamBus bus;
    Rv32Cpu cpu(bus);
    for (uint32_t i = 0; i < 200; ++i) bus.put32(0x40000000 + 4 * i, 0x00140413);  // addi s0, s0, 1
    cpu.reset(0x40000000);
    cpu.set_cycle_cost(2);
    const uint64_t c0 = cpu.cycles;
    CHECK(cpu.run(37) == 37);
    CHECK(cpu.x[8] == 37);
    CHECK(cpu.pc == 0x40000000 + 4 * 37);
    CHECK(cpu.instret == 37);
    CHECK(cpu.cycles - c0 == 74);
    CHECK(cpu.run(100) == 100);
    CHECK(cpu.x[8] == 137);
    CHECK(cpu.instret == 137);
}

static void test_compressed_decode() {
    uint32_t out;
    CHECK(Rv32Cpu::expand_compressed(0x0000, &out) == 0);  // illegal
    CHECK(Rv32Cpu::expand_compressed(0x4501, &out) && out == 0x00000513);  // c.li a0,0
    CHECK(Rv32Cpu::expand_compressed(0x1141, &out) && out == 0xff010113);  // c.addi sp,-16
    CHECK(Rv32Cpu::expand_compressed(0xc606, &out) && out == 0x00112623);  // c.swsp ra,12(sp)
    CHECK(Rv32Cpu::expand_compressed(0x40b2, &out) && out == 0x00c12083);  // c.lwsp ra,12(sp)
    CHECK(Rv32Cpu::expand_compressed(0x8082, &out) && out == 0x00008067);  // c.jr ra (ret)
    CHECK(Rv32Cpu::expand_compressed(0x0808, &out) && out == 0x01010513);  // c.addi4spn a0,sp,16
}

static void test_md5_inflate() {
    CHECK(Md5::hex(reinterpret_cast<const uint8_t*>("abc"), 3) == "900150983cd24fb0d6963f7d28e17f72");
    CHECK(Md5::hex(nullptr, 0) == "d41d8cd98f00b204e9800998ecf8427e");
    // zlib.compress(b"hello hello hello hello")
    const uint8_t z[] = {0x78, 0x9c, 0xcb, 0x48, 0xcd, 0xc9, 0xc9, 0x57, 0xc8, 0x40, 0x27, 0x01,
                         0x68, 0x03, 0x08, 0xb1};
    std::vector<uint8_t> out;
    bool ok = zlib_inflate(z, sizeof z, out);
    CHECK(ok);
    CHECK(std::string(out.begin(), out.end()) == "hello hello hello hello");
}

// Drive the SHA accelerator the way ESP-IDF's sha_ll does: message words
// written as little-endian loads of the byte stream, digest read back the same.
static std::string sha_hw(int mode, const std::string& msg) {
    c3::Sha sha("SHA");
    std::vector<uint8_t> m(msg.begin(), msg.end());
    uint64_t bits = uint64_t(m.size()) * 8;
    m.push_back(0x80);
    while (m.size() % 64 != 56) m.push_back(0);
    for (int i = 7; i >= 0; --i) m.push_back(uint8_t(bits >> (8 * i)));
    sha.write(0x00, uint32_t(mode));
    for (size_t blk = 0; blk < m.size(); blk += 64) {
        for (int w = 0; w < 16; ++w) {
            uint32_t v;
            std::memcpy(&v, &m[blk + size_t(w) * 4], 4);
            sha.write(0x80 + uint32_t(w) * 4, v);
        }
        sha.write(blk == 0 ? 0x10 : 0x14, 1);
    }
    int words = mode == 0 ? 5 : mode == 1 ? 7 : 8;
    std::string hex;
    for (int w = 0; w < words; ++w) {
        uint32_t v = sha.read(0x40 + uint32_t(w) * 4);
        char b[3];
        for (int i = 0; i < 4; ++i) {
            std::snprintf(b, sizeof b, "%02x", (v >> (8 * i)) & 0xFF);
            hex += b;
        }
    }
    return hex;
}

static void test_sha() {
    CHECK(sha_hw(2, "abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(sha_hw(1, "abc") == "23097d223405d8228642a477bda255b32aadbce4bda0b3f7e36c9da7");
    CHECK(sha_hw(0, "abc") == "a9993e364706816aba3e25717850c26c9cd0d89d");
    CHECK(sha_hw(2, std::string(100, 'a')) == "2816597888e4a0d3a36b82b83316ab32680eb8f00f8cd3b904d681246d285a0e");
}

static void test_intmatrix() {
    c3::IntMatrix im("INTMATRIX", c3::irq::kSources, true);
    uint32_t lines = 0;
    im.on_lines = [&](uint32_t l) { lines = l; };
    im.write(c3::irq::SYSTIMER_TARGET0 * 4, 7);  // source -> CPU line 7
    im.write(0x114 + 7 * 4, 3);                 // priority 3
    im.write(0x104, 1u << 7);                   // enable
    im.set_source(c3::irq::SYSTIMER_TARGET0, true);
    CHECK(lines == (1u << 7));
    im.write(0x194, 4);  // threshold above the priority masks it
    CHECK(lines == 0);
    im.write(0x194, 3);  // equal priority is delivered
    CHECK(lines == (1u << 7));
    im.set_source(c3::irq::SYSTIMER_TARGET0, false);
    CHECK(lines == 0);
}

// ESP32-C6: the PLIC reaches the line controller, map registers sit in the
// separate matrix block, and the CPU clock follows PCR.
static void test_c6_interrupts_and_clocks() {
    c3::IntMatrix im("INTMTX", c6::irq::kSources, false);
    c6::Plic plic("PLIC_MX", im);
    uint32_t lines = 0;
    im.on_lines = [&](uint32_t l) { lines = l; };
    im.write(c6::irq::UART0 * 4, 9);  // UART0 -> CPU line 9
    plic.write(0x10 + 9 * 4, 2);       // PLIC_MXINT9_PRI
    plic.write(0x90, 1);               // PLIC_MXINT_THRESH
    plic.write(0x00, 1u << 9);         // PLIC_MXINT_ENABLE
    CHECK(plic.read(0x00) == (1u << 9));
    im.set_source(c6::irq::UART0, true);
    CHECK(lines == (1u << 9));
    plic.write(0x90, 3);  // threshold above the line's priority masks it
    CHECK(lines == 0);

    c6::Pcr pcr("PCR");
    pcr.reset();
    CHECK(pcr.cpu_hz() == 40000000);   // XTAL after reset
    pcr.write(0x110, 1u << 16);        // SOC_CLK_SEL = PLL (HS_DIV_NUM stays 2)
    CHECK(pcr.cpu_hz() == 160000000);
    pcr.write(0x118, 1u << 8);         // CPU_HS_DIV_NUM = 1
    CHECK(pcr.cpu_hz() == 80000000);
    CHECK((pcr.read(0x110) >> 24) == 40);  // CLK_XTAL_FREQ is read-only
}

// ESP32-C6 flash MMU: entries are written through ITEM_INDEX/ITEM_CONTENT and
// the page size comes from MMU_POWER_CTRL.
static void test_c6_mmu() {
    c6::Spi0Mmu mmu("SPI0");
    std::vector<int> changes;
    mmu.on_change = [&](int i) { changes.push_back(i); };
    mmu.reset();
    CHECK(mmu.page_size() == 0x10000);
    mmu.write(0x380, 5);                          // ITEM_INDEX
    mmu.write(0x37C, c6::Spi0Mmu::kValid | 3);    // ITEM_CONTENT
    CHECK(mmu.entry(5) == (c6::Spi0Mmu::kValid | 3));
    CHECK(mmu.read(0x37C) == (c6::Spi0Mmu::kValid | 3));
    mmu.write(0x384, 1u << 3);  // MMU_PAGE_SIZE = 1: 32 KiB
    CHECK(mmu.page_size() == 0x8000);
    CHECK(changes.size() == 3 && changes[0] == -1 && changes[1] == 5 && changes[2] == -1);
    CHECK(mmu.read(0x170) >> 26 == 0x3F);  // AXI FIFOs report empty

    c6::I2cAnaMst ana("I2C_ANA_MST");
    ana.write(0x00, (1u << 24) | (0x5Au << 16) | (0x07u << 8) | 0x66);  // write BBPLL reg 7
    ana.write(0x00, (0x07u << 8) | 0x66);                               // read it back
    CHECK(((ana.read(0x00) >> 16) & 0xFF) == 0x5A);
    CHECK(ana.read(0x18) & (1u << 24));  // BBPLL calibration done
}

int main() {
    test_sha();
    test_intmatrix();
    test_c6_interrupts_and_clocks();
    test_c6_mmu();
    test_base_isa();
    test_compressed_loop();
    test_trap_and_mret();
    test_self_modifying_code();
    test_run_budget();
    test_compressed_decode();
    test_md5_inflate();
    if (g_failures) {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("all tests passed\n");
    return 0;
}
