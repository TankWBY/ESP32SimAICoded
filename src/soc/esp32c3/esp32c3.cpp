#include "soc/esp32c3/esp32c3.hpp"

#include <algorithm>
#include <cstdio>

#include "soc/esp32c3/rom_hle.hpp"
#include "util/log.hpp"

namespace esp32sim {

using namespace c3map;

Esp32c3::Esp32c3(const ChipConfig& cfg)
    : cfg_(cfg),
      cpu_(*this),
      flash_(cfg.flash_size),
      sram_(IRAM_SIZE, 0),
      rtc_fast_(kPageSize, 0),
      rom_image_(ROM_SIZE, 0),
      drom_image_(DROM_MASK_SIZE, 0) {
    flash_.load(cfg.flash_path);
    flash_.on_modified = [this] { if (state_ == State::Running) cpu_.flush_icache(); };

    map(IRAM_BASE, sram_.data(), IRAM_SIZE, true);
    map(DRAM_BASE, sram_.data() + (SRAM1_IRAM - IRAM_BASE), DRAM_SIZE, true);
    map(RTC_FAST_BASE, rtc_fast_.data(), kPageSize, true);
    if (!cfg.rom_path.empty()) {
        std::string err;
        if (!rom_elf_.load(cfg.rom_path, &err)) {
            LOGE("rom: %s; continuing with the built-in ROM emulation only", err.c_str());
        } else {
            for (const auto& sec : rom_elf_.sections()) {
                if (sec.nobits) continue;
                if (sec.addr - ROM_BASE < ROM_SIZE && sec.addr + sec.size <= ROM_BASE + ROM_SIZE)
                    std::copy(sec.data.begin(), sec.data.end(), rom_image_.begin() + (sec.addr - ROM_BASE));
                else if (sec.addr - DROM_MASK_BASE < DROM_MASK_SIZE && sec.addr + sec.size <= DROM_MASK_BASE + DROM_MASK_SIZE)
                    std::copy(sec.data.begin(), sec.data.end(), drom_image_.begin() + (sec.addr - DROM_MASK_BASE));
            }
            have_rom_ = true;
            LOGI("rom: loaded %s (%zu symbols)", cfg.rom_path.c_str(), rom_elf_.symbols().size());
        }
    }
    map(ROM_BASE, rom_image_.data(), ROM_SIZE, false);
    map(DROM_MASK_BASE, drom_image_.data(), DROM_MASK_SIZE, false);

    uart0_ = std::make_unique<c3::Uart>("UART0");
    uart1_ = std::make_unique<c3::Uart>("UART1");
    gpio_ = std::make_unique<c3::Gpio>("GPIO");
    gpio_->trace = cfg.trace_gpio;
    clocks_.now = &cpu_.cycles;
    clocks_.master_hz = cfg.cpu_hz;
    systimer_ = std::make_unique<c3::Systimer>("SYSTIMER", clocks_);
    timg0_ = std::make_unique<c3::Timg>("TIMG0", clocks_, c3::irq::TG0_T0, c3::irq::TG0_WDT);
    timg1_ = std::make_unique<c3::Timg>("TIMG1", clocks_, c3::irq::TG1_T0, c3::irq::TG1_WDT);
    system_ = std::make_unique<c3::SystemReg>("SYSTEM");
    intmatrix_ = std::make_unique<c3::IntMatrix>("INTMATRIX");
    timed_ = {systimer_.get(), timg0_.get(), timg1_.get()};
    efuse_ = std::make_unique<c3::Efuse>("EFUSE", cfg.mac);
    rtc_cntl_ = std::make_unique<c3::RtcCntl>("RTC_CNTL");
    mmu_ = std::make_unique<c3::Mmu>("MMU");
    usb_jtag_ = std::make_unique<c3::UsbSerialJtag>("USB_SERIAL_JTAG");
    extmem_ = std::make_unique<c3::Extmem>("EXTMEM");
    syscon_ = std::make_unique<c3::Syscon>("SYSCON");
    sha_ = std::make_unique<c3::Sha>("SHA");
    spi1_ = std::make_unique<SpiMemController>("SPI1", flash_);

    uart0_->on_tx = [this](uint8_t b) { if (!cfg_.usb_console) tx_buf_.push_back(b); };
    uart1_->on_tx = [](uint8_t b) { LOGD("uart1 tx: 0x%02x", b); };
    usb_jtag_->on_tx = [this](uint8_t b) { if (cfg_.usb_console) tx_buf_.push_back(b); };
    rtc_cntl_->on_system_reset = [this](double delay) { request_reset(BootMode::Flash, delay); };
    rtc_cntl_->slow_ticks = [this] {
        return uint64_t(double(cpu_.cycles) * c3::RtcCntl::kSlowClockHz / clocks_.master_hz);
    };
    rtc_cntl_->on_wdt = [this](double delay) {
        wdt_armed_ = delay >= 0;
        if (wdt_armed_) wdt_at_cycles_ = cpu_.cycles + uint64_t(delay * cfg_.cpu_hz);
    };
    mmu_->on_change = [this](int i, uint32_t v) { on_mmu_change(i, v); };
    auto irq = [this](int src, bool level) { intmatrix_->set_source(src, level); };
    systimer_->irq = irq;
    gpio_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::GPIO, l); };
    uart0_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::UART0, l); };
    uart1_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::UART1, l); };
    timg0_->irq = irq;
    timg1_->irq = irq;
    timg0_->on_wdt_reset = [this](bool sys) { request_reset(BootMode::Flash, 0), rtc_cntl_->reset_reason = sys ? 7 : 11; };
    timg1_->on_wdt_reset = [this](bool sys) { request_reset(BootMode::Flash, 0), rtc_cntl_->reset_reason = sys ? 8 : 17; };
    system_->on_clock_change = [this] { apply_clocks(); };
    system_->on_cpu_intr = [this](int i, bool level) { intmatrix_->set_source(c3::irq::FROM_CPU0 + i, level); };
    intmatrix_->on_lines = [this](uint32_t lines) { cpu_.set_irq_lines(lines); };
    cpu_.mie_bypass = ~1u;

    attach(UART0, uart0_.get());
    attach(UART1, uart1_.get());
    attach(SPI1, spi1_.get());
    attach(GPIO, gpio_.get());
    attach(IO_MUX, gpio_->iomux());
    attach(RTC_CNTL, rtc_cntl_.get(), 0x800);
    // EFUSE shares RTC_CNTL's 4 KiB page; slot_for() routes it via efuse_slot_.
    attach(SYSTIMER, systimer_.get(), 0x1000, true);
    attach(TIMG0, timg0_.get(), 0x1000, true);
    attach(TIMG1, timg1_.get(), 0x1000, true);
    attach(SYSTEM, system_.get());
    attach(INTMATRIX, intmatrix_.get());
    attach(USB_SERIAL_JTAG, usb_jtag_.get());
    attach(MMU_TABLE, mmu_.get());
    attach(EXTMEM, extmem_.get());
    attach(SYSCON, syscon_.get());
    attach(SHA, sha_.get());
    efuse_slot_ = {efuse_.get(), EFUSE};

    cpu_.on_fatal_trap = [this](Rv32Cpu&, uint32_t cause, uint32_t tval) { on_fatal_trap(cause, tval); };
    cpu_.hook = [this](Rv32Cpu& c) { return on_rom_call(c); };
    cpu_.hook_lo = ROM_BASE;
    cpu_.hook_hi = have_rom_ ? ROM_BASE : ROM_BASE + ROM_SIZE;  // real ROM code runs natively
    // PMP, counter-enable and ESP32-C3 vendor CSRs (performance counters,
    // GPIO-on-CSR): plain storage, no behaviour yet.
    cpu_.custom_csr = [this](uint32_t csr, uint32_t* v, bool write) {
        bool known = (csr >= 0x3A0 && csr <= 0x3EF) || csr == 0x306 || csr == 0x320 || (csr >= 0x7C0 && csr <= 0x8FF) ||
                     (csr >= 0xBC0 && csr <= 0xBFF);
        if (!known) return false;
        if (csr == 0x7E2 || csr == 0x802) {  // MPCCR / UPCCR: performance counter, counts cycles
            if (write) mpccr_offset_ = *v - uint32_t(cpu_.core_cycles());
            else *v = uint32_t(cpu_.core_cycles()) + mpccr_offset_;
            return true;
        }
        if (write) extra_csrs_[csr] = *v;
        else *v = extra_csrs_.count(csr) ? extra_csrs_[csr] : 0;
        return true;
    };

    rom_ = std::make_unique<Esp32c3Rom>(*this);
}

Esp32c3::~Esp32c3() = default;

void Esp32c3::attach(uint32_t base, Peripheral* p, uint32_t size, bool timed) {
    for (uint32_t off = 0; off < size; off += 0x1000) {
        uint32_t idx = (base + off - PERIPH_BASE) >> 12;
        periph_map_[idx] = {p, base, timed};
    }
}

Esp32c3::PeriphSlot& Esp32c3::create_generic_slot(uint32_t idx) {
    char name[32];
    std::snprintf(name, sizeof name, "periph@%08x", PERIPH_BASE + (idx << 12));
    generic_.push_back(std::make_unique<RegisterFile>(name));
    periph_map_[idx] = {generic_.back().get(), PERIPH_BASE + (idx << 12)};
    LOGD("mmio: first access to unmodelled block %s (acting as plain registers)", name);
    return periph_map_[idx];
}

inline Esp32c3::PeriphSlot& Esp32c3::slot_for(uint32_t addr) {
    // RTC_CNTL (0x6000_8000) and EFUSE (0x6000_8800) share one 4 KiB page.
    if (addr - EFUSE < 0x800) return efuse_slot_;
    uint32_t idx = (addr - PERIPH_BASE) >> 12;
    PeriphSlot& s = periph_map_[idx];
    return s.p ? s : create_generic_slot(idx);
}

uint32_t Esp32c3::mmio_read(uint32_t addr, int size) {
    if (addr - PERIPH_BASE < PERIPH_SIZE) {
        const PeriphSlot& s = slot_for(addr);
        uint32_t word = s.p->read((addr & ~3u) - s.base);
        uint32_t v = word >> (8 * (addr & 3));
        if (size == 1) return v & 0xFF;
        if (size == 2) return v & 0xFFFF;
        return v;
    }
    uint32_t page = addr >> 16;
    if (warned_pages_.insert(page).second)
        LOGW("bus: read from unmapped address 0x%08x (pc=0x%08x)", addr, cpu_.pc);
    return 0;
}

void Esp32c3::mmio_write(uint32_t addr, uint32_t value, int size) {
    if (addr - PERIPH_BASE < PERIPH_SIZE) {
        const PeriphSlot& s = slot_for(addr);
        uint32_t off = (addr & ~3u) - s.base;
        if (size == 4 || (addr & 3) == 0) {
            if (size == 1) value &= 0xFF;
            if (size == 2) value &= 0xFFFF;
            s.p->write(off, value);
        } else {
            uint32_t sh = 8 * (addr & 3);
            uint32_t mask = (size == 1 ? 0xFFu : 0xFFFFu) << sh;
            uint32_t old = s.p->read(off);
            s.p->write(off, (old & ~mask) | ((value << sh) & mask));
        }
        if (s.timed) cpu_.request_stop();  // re-plan the slice around the new deadline
        return;
    }
    uint32_t page = addr >> 16;
    if (warned_pages_.insert(page).second)
        LOGW("bus: write to unmapped address 0x%08x (pc=0x%08x)", addr, cpu_.pc);
}

void Esp32c3::on_mmu_change(int index, uint32_t value) {
    cpu_.flush_icache();
    const uint32_t off = uint32_t(index) * kPageSize;
    uint32_t page = value & 0xFF;
    if ((value & c3::Mmu::kInvalid) || (page + 1) * kPageSize > flash_.size()) {
        unmap(IROM_BASE + off, kPageSize);
        unmap(DROM_BASE + off, kPageSize);
        return;
    }
    uint8_t* host = flash_.data() + page * kPageSize;
    map(IROM_BASE + off, host, kPageSize, false);
    map(DROM_BASE + off, host, kPageSize, false);
}

void Esp32c3::reset(BootMode mode) {
    // GPIO9 is the boot strap: held low from outside (the BOOT button) while
    // EN is released, the ROM enters download mode.
    if (mode == BootMode::Flash && gpio_->info(9).external == 0) {
        LOGI("reset: GPIO9 held low (BOOT button) -> download mode");
        mode = BootMode::Download;
    }
    do_reset(mode, 1 /* POWERON: EN pin */);
}

void Esp32c3::do_reset(BootMode mode, uint32_t reason) {
    reset_requested_ = false;
    wdt_armed_ = false;
    // Power-on and RTC watchdog resets clear the RTC domain; software and CPU
    // resets keep it (the STORE registers carry state across those).
    if (reason == 1 || reason == 16) rtc_cntl_->reset();
    rtc_cntl_->reset_reason = reason;
    cpu_.reset(0x40000400);  // architectural reset vector inside mask ROM
    uart0_->reset();
    uart1_->reset();
    gpio_->reset();
    gpio_->set_strap(mode == BootMode::Download ? 0x4 : 0xC);
    mpccr_offset_ = 0;
    intmatrix_->reset();
    system_->reset();  // back to the XTAL clock
    systimer_->reset();
    timg0_->reset();
    timg1_->reset();
    spi1_->reset();
    usb_jtag_->reset();
    extmem_->reset();
    syscon_->reset();
    sha_->reset();
    mmu_->reset();
    for (auto& g : generic_) g->reset();
    host_rx_.clear();
    if (have_rom_) load_rom_data();
    state_ = State::Loader;  // the ROM decides what happens next
    rom_->on_reset(mode);
    flush_output();
}

void Esp32c3::hold_in_reset() {
    if (state_ != State::InReset) LOGD("chip held in reset (EN low)");
    state_ = State::InReset;
    host_rx_.clear();
}

// The mask ROM's startup code initialises its .data/.bss in SRAM; with the
// boot itself emulated at a high level we do that from the ROM ELF.
void Esp32c3::load_rom_data() {
    for (const auto& sec : rom_elf_.sections()) {
        if (sec.addr - DRAM_BASE >= DRAM_SIZE) continue;
        uint8_t* dst = sram_.data() + (SRAM1_IRAM - IRAM_BASE) + (sec.addr - DRAM_BASE);
        if (sec.addr + sec.size > DRAM_BASE + DRAM_SIZE) continue;
        if (sec.nobits) std::fill(dst, dst + sec.size, 0);
        else std::copy(sec.data.begin(), sec.data.end(), dst);
    }
    // The ROM's startup code enables its UART console (ets_install_uart_printf);
    // uart_tx_one_char() drops characters until it has.
    for (const auto& [addr, name] : rom_elf_.symbols())
        if (name == "g_uart_print") write<1>(addr, 1);
}

std::string Esp32c3::where() const {
    char buf[48];
    std::snprintf(buf, sizeof buf, "pc=0x%08x", cpu_.pc);
    std::string s = buf;
    std::string sym = symbolize(cpu_.pc);
    if (!sym.empty()) s += " <" + sym + ">";
    std::string ra = symbolize(cpu_.x[1]);
    if (!ra.empty()) s += " ra <" + ra + ">";
    return s;
}

void Esp32c3::add_symbols(const std::string& elf_path) {
    ElfFile f;
    std::string err;
    if (f.load(elf_path, &err)) symbol_files_.push_back(std::move(f));
    else LOGW("symbols: %s", err.c_str());
}

std::string Esp32c3::symbolize(uint32_t addr) const {
    uint32_t off = 0, best_off = ~0u;
    std::string best;
    auto consider = [&](const ElfFile& f) {
        std::string n = f.symbolize(addr, &off);
        if (!n.empty() && off < best_off && off < 0x10000) { best = n; best_off = off; }
    };
    if (have_rom_ && addr - ROM_BASE < ROM_SIZE) consider(rom_elf_);
    for (const auto& f : symbol_files_) consider(f);
    if (best.empty()) return {};
    char buf[16];
    std::snprintf(buf, sizeof buf, "+0x%x", best_off);
    return best + buf;
}

std::vector<int> Esp32c3::user_gpios() const {
    // GPIO11-17 belong to the SPI flash on every C3 module.
    return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 18, 19, 20, 21};
}

uint32_t Esp32c3::reset_reason() const { return rtc_cntl_->reset_reason; }

void Esp32c3::request_reset(BootMode mode, double delay_s) {
    reset_requested_ = true;
    requested_mode_ = mode;
    reset_at_cycles_ = cpu_.cycles + uint64_t(delay_s * cfg_.cpu_hz);
    if (delay_s <= 0) cpu_.request_stop();  // end the current run() slice promptly
}

void Esp32c3::apply_clocks() {
    uint32_t hz = system_->cpu_hz();
    cpu_.set_cycle_cost(std::max<uint32_t>(1, (clocks_.master_hz + hz / 2) / hz));
    clocks_.apb_hz = system_->apb_hz();
    LOGD("clock: CPU %u MHz, APB %u MHz", hz / 1000000, clocks_.apb_hz / 1000000);
}

uint64_t Esp32c3::next_timer_event() const {
    uint64_t t = c3::TimedDevice::kNever;
    for (auto* d : timed_) t = std::min(t, d->next_event());
    return t;
}

void Esp32c3::tick_timers() {
    for (auto* d : timed_)
        if (d->next_event() <= cpu_.cycles) d->tick();
}

// Idle time: jump to the next scheduled event (pending reset, watchdog) or the
// end of the slice, whichever comes first.
uint64_t Esp32c3::next_deadline(uint64_t target) const {
    uint64_t t = target;
    if (reset_requested_) t = std::min(t, std::max(cpu_.cycles, reset_at_cycles_));
    if (wdt_armed_ && state_ != State::InReset) t = std::min(t, std::max(cpu_.cycles, wdt_at_cycles_));
    return t;
}

void Esp32c3::start_cpu(uint32_t entry) {
    cpu_.reset(entry);
    cpu_.x[2] = ROM_STACK_TOP;  // the ROM jumps to the entry point on its own stack
    if (have_rom_) {
        // State the real ROM's startup code leaves behind: its vector table
        // and dispatcher (ets_isr_attach handlers), interrupts globally on,
        // threshold 1, and UART0 routed to CPU line 5 for the download-mode
        // receive ISR (line still masked). Code loaded with MEM_END, such as
        // esptool's flasher stub, relies on this.
        cpu_.mtvec = ROM_VECTOR_TABLE | 1;
        cpu_.mstatus |= 0x8;  // MIE
        intmatrix_->write(c3::irq::UART0 * 4, 5);
        intmatrix_->write(0x114 + 5 * 4, 1);  // CPU_INT_PRI_5
        intmatrix_->write(0x194, 1);          // CPU_INT_THRESH
    }
    state_ = State::Running;
    LOGI("cpu: running from 0x%08x", entry);
}

std::string Esp32c3::status() const {
    switch (state_) {
    case State::InReset: return "held in reset";
    case State::Loader: return "download mode";
    case State::Running: return "running";
    case State::Halted: return "halted: " + cpu_.halt_reason;
    }
    return {};
}

void Esp32c3::halt(const std::string& why) {
    cpu_.halted = true;
    cpu_.request_stop();
    cpu_.halt_reason = why;
    state_ = State::Halted;
    LOGE("cpu: halted: %s", why.c_str());
}

void Esp32c3::console_write(const std::string& s) { console_write(reinterpret_cast<const uint8_t*>(s.data()), s.size()); }
void Esp32c3::console_write(const uint8_t* data, size_t len) { tx_buf_.insert(tx_buf_.end(), data, data + len); }

void Esp32c3::flush_output() {
    if (tx_buf_.empty()) return;
    if (serial_output) serial_output(tx_buf_.data(), tx_buf_.size());
    tx_buf_.clear();
}

void Esp32c3::feed_uart_rx() {
    size_t n = uart0_->rx_space();
    while (n-- && !host_rx_.empty()) {
        uart0_->push_rx(host_rx_.front());
        host_rx_.pop_front();
    }
}

void Esp32c3::serial_input(const uint8_t* data, size_t len) {
    if (state_ == State::InReset || state_ == State::Halted) return;
    host_rx_.insert(host_rx_.end(), data, data + len);
}

void Esp32c3::run_for(uint64_t cycles) {
    const uint64_t target = cpu_.cycles + cycles;
    while (cpu_.cycles < target) {
        if (reset_requested_ && cpu_.cycles >= reset_at_cycles_) do_reset(requested_mode_, rtc_cntl_->reset_reason);
        if (wdt_armed_ && cpu_.cycles >= wdt_at_cycles_) {
            LOGI("rtc: RTC watchdog expired, resetting chip");
            do_reset(BootMode::Flash, 16);  // RTCWDT_RTC_RESET
        }
        if (state_ == State::Running) {
            feed_uart_rx();
            uint64_t slice = target - cpu_.cycles;
            if (slice > 16000) slice = 16000;
            if (wdt_armed_ && wdt_at_cycles_ > cpu_.cycles) slice = std::min(slice, wdt_at_cycles_ - cpu_.cycles);
            uint64_t ev = next_timer_event();
            if (ev <= cpu_.cycles) {
                tick_timers();
                ev = next_timer_event();
            }
            if (ev > cpu_.cycles) slice = std::min(slice, ev - cpu_.cycles);
            cpu_.run(std::max<uint64_t>(1, slice / cpu_.cycle_cost));
            tick_timers();
            if (cpu_.halted && !reset_requested_ && state_ == State::Running) {
                state_ = State::Halted;
                LOGE("cpu: halted: %s", cpu_.halt_reason.c_str());
            }
            if (state_ != State::Running && cpu_.cycles < target) cpu_.cycles = target;
        } else if (state_ == State::Loader) {
            if (!host_rx_.empty()) {
                std::vector<uint8_t> in(host_rx_.begin(), host_rx_.end());
                host_rx_.clear();
                rom_->feed(in.data(), in.size());
                flush_output();
                continue;  // the loader may have started the CPU
            }
            cpu_.cycles = next_deadline(target);
        } else {
            cpu_.cycles = next_deadline(target);
        }
    }
    flush_output();
    if (flash_.dirty() && cpu_.cycles >= flash_save_at_) {
        flash_.save();
        flash_save_at_ = cpu_.cycles + uint64_t(cfg_.cpu_hz) * 2;
    }
}

void Esp32c3::shutdown() {
    flush_output();
    flash_.save();
}

namespace {
const char* cause_name(uint32_t c) {
    switch (c) {
    case 0: return "instruction address misaligned";
    case 1: return "instruction access fault";
    case 2: return "illegal instruction";
    case 3: return "breakpoint";
    case 4: return "load address misaligned";
    case 5: return "load access fault";
    case 6: return "store address misaligned";
    case 7: return "store access fault";
    case 11: return "environment call (ecall)";
    default: return "unknown";
    }
}
}  // namespace

void Esp32c3::on_fatal_trap(uint32_t cause, uint32_t tval) {
    static const char* abi[32] = {"zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0",
                                  "a1",   "a2", "a3", "a4", "a5", "a6", "a7", "s2", "s3", "s4", "s5",
                                  "s6",   "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"};
    char buf[256];
    std::string sym = symbolize(cpu_.pc);
    std::snprintf(buf, sizeof buf, "unhandled exception %u (%s) at pc=0x%08x%s%s%s, mtval=0x%08x (no trap vector installed)",
                  cause, cause_name(cause), cpu_.pc, sym.empty() ? "" : " <", sym.c_str(), sym.empty() ? "" : ">", tval);
    state_ = State::Halted;
    cpu_.halt_reason = buf;
    LOGE("cpu: %s", buf);
    std::string ra = symbolize(cpu_.x[1]);
    if (!ra.empty()) LOGE("  ra is in %s", ra.c_str());
    for (int i = 0; i < 32; i += 4)
        LOGE("  %-4s=%08x %-4s=%08x %-4s=%08x %-4s=%08x", abi[i], cpu_.x[i], abi[i + 1], cpu_.x[i + 1], abi[i + 2],
             cpu_.x[i + 2], abi[i + 3], cpu_.x[i + 3]);
}

bool Esp32c3::on_rom_call(Rv32Cpu& c) {
    if (rom_->call(c)) return true;
    char buf[220];
    std::snprintf(buf, sizeof buf,
                  "firmware called mask-ROM routine at 0x%08x (ra=0x%08x) but no ROM image is loaded; "
                  "use --rom roms/esp32c3_rev3_rom.elf (tools/fetch_roms.py)",
                  c.pc, c.x[1]);
    halt(buf);
    return false;
}

}  // namespace esp32sim
