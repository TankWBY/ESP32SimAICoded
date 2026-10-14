#include "soc/riscv_soc.hpp"

#include <algorithm>
#include <cstdio>

#include "util/log.hpp"

namespace esp32sim {

RiscvSoc::RiscvSoc(const ChipConfig& cfg, uint32_t periph_base, uint32_t periph_size)
    : cfg_(cfg),
      cpu_(*this),
      flash_(cfg.flash_size),
      periph_base_(periph_base),
      periph_size_(periph_size),
      periph_map_(periph_size >> 10) {
    flash_.load(cfg.flash_path);
    flash_.on_modified = [this] { if (state_ == State::Running) cpu_.flush_icache(); };
    clocks_.now = &cpu_.cycles;
    clocks_.master_hz = cfg.cpu_hz;

    if (!cfg.rom_path.empty()) {
        std::string err;
        if (rom_elf_.load(cfg.rom_path, &err)) {
            have_rom_ = true;
            LOGI("rom: loaded %s (%zu symbols)", cfg.rom_path.c_str(), rom_elf_.symbols().size());
        } else {
            LOGE("rom: %s; continuing with the built-in ROM emulation only", err.c_str());
        }
    }

    cpu_.on_fatal_trap = [this](Rv32Cpu&, uint32_t cause, uint32_t tval) { on_fatal_trap(cause, tval); };
    cpu_.hook = [this](Rv32Cpu& c) { return on_rom_call(c); };
    // PMP/PMA, counter-enable and Espressif vendor CSRs (performance
    // counters, GPIO-on-CSR): plain storage, no behaviour yet.
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
}

RiscvSoc::~RiscvSoc() = default;

void RiscvSoc::load_rom(uint32_t base, std::vector<uint8_t>& image) {
    if (!have_rom_) return;
    for (const auto& sec : rom_elf_.sections()) {
        if (sec.nobits || sec.addr < base || sec.addr + sec.size > base + image.size()) continue;
        std::copy(sec.data.begin(), sec.data.end(), image.begin() + (sec.addr - base));
    }
}

void RiscvSoc::finish_setup(RomProfile profile, uint32_t rom_base, uint32_t rom_size) {
    rom_base_ = rom_base;
    rom_size_ = rom_size;
    cpu_.hook_lo = rom_base;
    cpu_.hook_hi = have_rom_ ? rom_base : rom_base + rom_size;  // real ROM code runs natively
    rom_ = std::make_unique<EspRom>(*this, std::move(profile));
}

void RiscvSoc::attach(uint32_t base, Peripheral* p, uint32_t size, bool timed) {
    for (uint32_t off = 0; off < size; off += 0x400) periph_map_[(base + off - periph_base_) >> 10] = {p, base, timed};
}

void RiscvSoc::attach_region(uint32_t base, uint32_t size, Peripheral* p) { regions_.push_back({base, size, p}); }

RiscvSoc::PeriphSlot& RiscvSoc::create_generic_slot(uint32_t idx) {
    // Unmodelled blocks become plain register files, one per 4 KiB page.
    uint32_t first = idx & ~3u;
    while (first < idx && periph_map_[first].p) ++first;
    uint32_t base = periph_base_ + (first << 10);
    char name[32];
    std::snprintf(name, sizeof name, "periph@%08x", base);
    generic_.push_back(std::make_unique<RegisterFile>(name));
    for (uint32_t i = first; i < ((idx | 3u) + 1) && i < periph_map_.size(); ++i)
        if (!periph_map_[i].p) periph_map_[i] = {generic_.back().get(), base};
    LOGD("mmio: first access to unmodelled block %s (acting as plain registers)", name);
    return periph_map_[idx];
}

inline RiscvSoc::PeriphSlot& RiscvSoc::slot_for(uint32_t addr) {
    uint32_t idx = (addr - periph_base_) >> 10;
    PeriphSlot& s = periph_map_[idx];
    return s.p ? s : create_generic_slot(idx);
}

RiscvSoc::Region* RiscvSoc::region_for(uint32_t addr) {
    for (auto& r : regions_)
        if (addr - r.base < r.size) return &r;
    return nullptr;
}

namespace {
uint32_t narrow(uint32_t word, uint32_t addr, int size) {
    uint32_t v = word >> (8 * (addr & 3));
    if (size == 1) return v & 0xFF;
    if (size == 2) return v & 0xFFFF;
    return v;
}

void store(Peripheral* p, uint32_t off, uint32_t addr, uint32_t value, int size) {
    if (size == 4 || (addr & 3) == 0) {
        if (size == 1) value &= 0xFF;
        if (size == 2) value &= 0xFFFF;
        p->write(off, value);
        return;
    }
    uint32_t sh = 8 * (addr & 3);
    uint32_t mask = (size == 1 ? 0xFFu : 0xFFFFu) << sh;
    uint32_t old = p->read(off);
    p->write(off, (old & ~mask) | ((value << sh) & mask));
}
}  // namespace

uint32_t RiscvSoc::mmio_read(uint32_t addr, int size) {
    if (addr - periph_base_ < periph_size_) {
        const PeriphSlot& s = slot_for(addr);
        return narrow(s.p->read((addr & ~3u) - s.base), addr, size);
    }
    if (Region* r = region_for(addr)) return narrow(r->p->read((addr & ~3u) - r->base), addr, size);
    if (warned_pages_.insert(addr >> 16).second)
        LOGW("bus: read from unmapped address 0x%08x (pc=0x%08x)", addr, cpu_.pc);
    return 0;
}

void RiscvSoc::mmio_write(uint32_t addr, uint32_t value, int size) {
    if (addr - periph_base_ < periph_size_) {
        const PeriphSlot& s = slot_for(addr);
        store(s.p, (addr & ~3u) - s.base, addr, value, size);
        if (s.timed) cpu_.request_stop();  // re-plan the slice around the new deadline
        return;
    }
    if (Region* r = region_for(addr)) {
        store(r->p, (addr & ~3u) - r->base, addr, value, size);
        return;
    }
    if (warned_pages_.insert(addr >> 16).second)
        LOGW("bus: write to unmapped address 0x%08x (pc=0x%08x)", addr, cpu_.pc);
}

void RiscvSoc::reset(BootMode mode) {
    // The boot strapping pin held low from outside (the BOOT button) while EN
    // is released makes the ROM enter download mode.
    if (mode == BootMode::Flash && gpio_ && gpio_->info(boot_pin_).external == 0) {
        LOGI("reset: GPIO%d held low (BOOT button) -> download mode", boot_pin_);
        mode = BootMode::Download;
    }
    do_reset(mode, 1 /* POWERON: EN pin */);
}

void RiscvSoc::do_reset(BootMode mode, uint32_t reason) {
    reset_requested_ = false;
    wdt_armed_ = false;
    cpu_.reset(rom_base_ + 0x400);  // architectural reset vector inside mask ROM
    mpccr_offset_ = 0;
    reset_peripherals(mode, reason);
    set_reset_reason(reason);
    for (auto& g : generic_) g->reset();
    host_rx_.clear();
    if (have_rom_) load_rom_data();
    state_ = State::Loader;  // the ROM decides what happens next
    rom_->on_reset(mode);
    flush_output();
}

void RiscvSoc::hold_in_reset() {
    if (state_ != State::InReset) LOGD("chip held in reset (EN low)");
    state_ = State::InReset;
    host_rx_.clear();
}

// The mask ROM's startup code initialises its .data/.bss in SRAM; with the
// boot itself emulated at a high level we do that from the ROM ELF.
void RiscvSoc::load_rom_data() {
    for (const auto& sec : rom_elf_.sections()) {
        if (sec.addr - rom_base_ < rom_size_) continue;  // code and constants
        bool in_ram = false;
        for (const auto& r : rom_->profile().ram)
            if (sec.addr >= r.base && sec.addr + sec.size <= r.base + r.size) in_ram = true;
        if (!in_ram) continue;
        for (uint32_t i = 0; i < sec.size; ++i) write<1>(sec.addr + i, sec.nobits ? 0 : sec.data[i]);
    }
    init_rom_state();
}

uint32_t RiscvSoc::rom_symbol(const std::string& name) const {
    for (const auto& [addr, n] : rom_elf_.symbols())
        if (n == name) return addr;
    return 0;
}

void RiscvSoc::write_rom_word(const std::string& symbol, uint32_t value, int size) {
    uint32_t a = rom_symbol(symbol);
    if (!a) return;
    if (size == 1) write<1>(a, uint8_t(value));
    else write<4>(a, value);
}

std::string RiscvSoc::where() const {
    char buf[48];
    std::snprintf(buf, sizeof buf, "pc=0x%08x", cpu_.pc);
    std::string s = buf;
    std::string sym = symbolize(cpu_.pc);
    if (!sym.empty()) s += " <" + sym + ">";
    std::string ra = symbolize(cpu_.x[1]);
    if (!ra.empty()) s += " ra <" + ra + ">";
    return s;
}

void RiscvSoc::add_symbols(const std::string& elf_path) {
    ElfFile f;
    std::string err;
    if (f.load(elf_path, &err)) symbol_files_.push_back(std::move(f));
    else LOGW("symbols: %s", err.c_str());
}

std::string RiscvSoc::symbolize(uint32_t addr) const {
    uint32_t off = 0, best_off = ~0u;
    std::string best;
    auto consider = [&](const ElfFile& f) {
        std::string n = f.symbolize(addr, &off);
        if (!n.empty() && off < best_off && off < 0x10000) { best = n; best_off = off; }
    };
    if (have_rom_ && addr - rom_base_ < rom_size_) consider(rom_elf_);
    for (const auto& f : symbol_files_) consider(f);
    if (best.empty()) return {};
    char buf[16];
    std::snprintf(buf, sizeof buf, "+0x%x", best_off);
    return best + buf;
}

void RiscvSoc::request_reset(BootMode mode, double delay_s) {
    reset_requested_ = true;
    requested_mode_ = mode;
    reset_at_cycles_ = cpu_.cycles + uint64_t(delay_s * cfg_.cpu_hz);
    if (delay_s <= 0) cpu_.request_stop();  // end the current run() slice promptly
}

void RiscvSoc::arm_rtc_wdt(double delay_s) {
    wdt_armed_ = delay_s >= 0;
    if (wdt_armed_) wdt_at_cycles_ = cpu_.cycles + uint64_t(delay_s * cfg_.cpu_hz);
}

void RiscvSoc::apply_cpu_clock(uint32_t hz) {
    cpu_.set_cycle_cost(std::max<uint32_t>(1, (clocks_.master_hz + hz / 2) / hz));
    LOGD("clock: CPU %u MHz, APB %u MHz", hz / 1000000, clocks_.apb_hz / 1000000);
}

uint64_t RiscvSoc::next_timer_event() const {
    uint64_t t = c3::TimedDevice::kNever;
    for (auto* d : timed_) t = std::min(t, d->next_event());
    return t;
}

void RiscvSoc::tick_timers() {
    for (auto* d : timed_)
        if (d->next_event() <= cpu_.cycles) d->tick();
}

// Idle time: jump to the next scheduled event (pending reset, watchdog) or the
// end of the slice, whichever comes first.
uint64_t RiscvSoc::next_deadline(uint64_t target) const {
    uint64_t t = target;
    if (reset_requested_) t = std::min(t, std::max(cpu_.cycles, reset_at_cycles_));
    if (wdt_armed_ && state_ != State::InReset) t = std::min(t, std::max(cpu_.cycles, wdt_at_cycles_));
    return t;
}

void RiscvSoc::start_cpu(uint32_t entry) {
    cpu_.reset(entry);
    cpu_.x[2] = rom_stack_top();  // the ROM jumps to the entry point on its own stack
    if (have_rom_) on_cpu_start();
    state_ = State::Running;
    LOGI("cpu: running from 0x%08x", entry);
}

std::string RiscvSoc::status() const {
    switch (state_) {
    case State::InReset: return "held in reset";
    case State::Loader: return "download mode";
    case State::Running: return "running";
    case State::Halted: return "halted: " + cpu_.halt_reason;
    }
    return {};
}

void RiscvSoc::halt(const std::string& why) {
    cpu_.halted = true;
    cpu_.request_stop();
    cpu_.halt_reason = why;
    state_ = State::Halted;
    LOGE("cpu: halted: %s", why.c_str());
}

void RiscvSoc::console_write(const std::string& s) { tx_buf_.insert(tx_buf_.end(), s.begin(), s.end()); }

void RiscvSoc::flush_output() {
    if (tx_buf_.empty()) return;
    if (serial_output) serial_output(tx_buf_.data(), tx_buf_.size());
    tx_buf_.clear();
}

void RiscvSoc::feed_uart_rx() {
    c3::Uart& u = console_uart();
    size_t n = u.rx_space();
    while (n-- && !host_rx_.empty()) {
        u.push_rx(host_rx_.front());
        host_rx_.pop_front();
    }
}

void RiscvSoc::serial_input(const uint8_t* data, size_t len) {
    if (state_ == State::InReset || state_ == State::Halted) return;
    host_rx_.insert(host_rx_.end(), data, data + len);
}

void RiscvSoc::run_for(uint64_t cycles) {
    const uint64_t target = cpu_.cycles + cycles;
    while (cpu_.cycles < target) {
        if (reset_requested_ && cpu_.cycles >= reset_at_cycles_) do_reset(requested_mode_, reset_reason());
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

void RiscvSoc::shutdown() {
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

void RiscvSoc::on_fatal_trap(uint32_t cause, uint32_t tval) {
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

bool RiscvSoc::on_rom_call(Rv32Cpu& c) {
    if (rom_->call(c)) return true;
    char buf[240];
    std::snprintf(buf, sizeof buf,
                  "firmware called mask-ROM routine at 0x%08x (ra=0x%08x) but no ROM image is loaded; "
                  "use --rom %s (tools/fetch_roms.py)",
                  c.pc, c.x[1], rom_->profile().rom_file.c_str());
    halt(buf);
    return false;
}

}  // namespace esp32sim
