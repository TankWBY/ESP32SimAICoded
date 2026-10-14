#include "soc/esp32c6/esp32c6.hpp"

#include "soc/esp32c6/gpio_pads.hpp"
#include "util/log.hpp"

namespace esp32sim {

using namespace c6map;

Esp32c6::Esp32c6(const ChipConfig& cfg)
    : RiscvSoc(cfg, PERIPH_BASE, PERIPH_SIZE),
      hp_sram_(HP_SRAM_SIZE, 0),
      lp_sram_(LP_SRAM_SIZE, 0),
      rom_image_(ROM_SIZE, 0) {
    map(HP_SRAM_BASE, hp_sram_.data(), HP_SRAM_SIZE, true);
    map(LP_SRAM_BASE, lp_sram_.data(), LP_SRAM_SIZE, true);
    load_rom(ROM_BASE, rom_image_);
    map(ROM_BASE, rom_image_.data(), ROM_SIZE, false);

    clocks_.apb_hz = c3::Clocks::kXtalHz;  // TIMG and SYSTIMER run from XTAL on the C6

    uart0_ = std::make_unique<c3::Uart>("UART0");
    uart1_ = std::make_unique<c3::Uart>("UART1");
    gpio_dev_ = std::make_unique<c3::Gpio>("GPIO", c6::kPads);
    gpio_dev_->trace = cfg.trace_gpio;
    gpio_ = gpio_dev_.get();
    systimer_ = std::make_unique<c3::Systimer>("SYSTIMER", clocks_);
    timg0_ = std::make_unique<c3::Timg>("TIMG0", clocks_, c6::irq::TG0_T0, c6::irq::TG0_WDT);
    timg1_ = std::make_unique<c3::Timg>("TIMG1", clocks_, c6::irq::TG1_T0, c6::irq::TG1_WDT);
    timed_ = {systimer_.get(), timg0_.get(), timg1_.get()};
    intmtx_ = std::make_unique<c3::IntMatrix>("INTMTX", c6::irq::kSources, false);
    efuse_ = std::make_unique<c3::Efuse>("EFUSE", cfg.mac, 1u << 18);  // wafer minor 1: chip revision v0.1
    usb_jtag_ = std::make_unique<c3::UsbSerialJtag>("USB_SERIAL_JTAG");
    lpperi_ = std::make_unique<c3::Syscon>("LPPERI", 0x8);
    sha_ = std::make_unique<c3::Sha>("SHA");
    spi1_ = std::make_unique<SpiMemController>("SPI1", flash_);
    spi0_ = std::make_unique<c6::Spi0Mmu>("SPI0");
    pcr_ = std::make_unique<c6::Pcr>("PCR");
    lp_clkrst_ = std::make_unique<c6::LpClkRst>("LP_CLKRST");
    lp_aon_ = std::make_unique<c6::LpAon>("LP_AON");
    lp_wdt_ = std::make_unique<c6::LpWdt>("LP_WDT");
    lp_timer_ = std::make_unique<c6::LpTimer>("LP_TIMER");
    cache_ = std::make_unique<c6::Cache>("CACHE");
    i2c_ana_ = std::make_unique<c6::I2cAnaMst>("I2C_ANA_MST");
    plic_ = std::make_unique<c6::Plic>("PLIC_MX", *intmtx_);
    intpri_ = std::make_unique<c6::Intpri>("INTPRI");
    clint_ = std::make_unique<RegisterFile>("CLINT");

    uart0_->on_tx = [this](uint8_t b) { if (!cfg_.usb_console) tx_buf_.push_back(b); };
    uart1_->on_tx = [](uint8_t b) { LOGD("uart1 tx: 0x%02x", b); };
    usb_jtag_->on_tx = [this](uint8_t b) { if (cfg_.usb_console) tx_buf_.push_back(b); };
    lp_aon_->on_sw_reset = [this](uint32_t reason) {
        lp_clkrst_->reset_reason = reason;
        request_reset(BootMode::Flash, 0);
    };
    lp_wdt_->on_wdt = [this](double delay) { arm_rtc_wdt(delay); };
    lp_timer_->slow_ticks = [this] { return uint64_t(double(cpu_.cycles) * c6::kSlowClockHz / clocks_.master_hz); };
    spi0_->on_change = [this](int index) { remap_mmu(index); };
    pcr_->on_clock_change = [this] { apply_clocks(); };

    auto irq = [this](int src, bool level) { intmtx_->set_source(src, level); };
    systimer_->irq = [this](int unit, bool l) { intmtx_->set_source(c6::irq::SYSTIMER_TARGET0 + unit, l); };
    gpio_dev_->irq = [this](bool l) { intmtx_->set_source(c6::irq::GPIO, l); };
    uart0_->irq = [this](bool l) { intmtx_->set_source(c6::irq::UART0, l); };
    uart1_->irq = [this](bool l) { intmtx_->set_source(c6::irq::UART1, l); };
    timg0_->irq = irq;
    timg1_->irq = irq;
    timg0_->on_wdt_reset = [this](bool sys) {
        lp_clkrst_->reset_reason = sys ? c6::reset::TG0_WDT_SYS : c6::reset::TG0_WDT_CPU;
        request_reset(BootMode::Flash, 0);
    };
    timg1_->on_wdt_reset = [this](bool sys) {
        lp_clkrst_->reset_reason = sys ? c6::reset::TG1_WDT_SYS : c6::reset::TG1_WDT_CPU;
        request_reset(BootMode::Flash, 0);
    };
    intpri_->on_cpu_intr = [this](int i, bool l) { intmtx_->set_source(c6::irq::FROM_CPU0 + i, l); };
    intmtx_->on_lines = [this](uint32_t lines) { cpu_.set_irq_lines(lines); };
    cpu_.mie_bypass = ~1u;  // lines are enabled in the PLIC, not in mie

    attach(UART0, uart0_.get());
    attach(UART1, uart1_.get());
    attach(SPI0, spi0_.get());
    attach(SPI1, spi1_.get());
    attach(TIMG0, timg0_.get(), 0x1000, true);
    attach(TIMG1, timg1_.get(), 0x1000, true);
    attach(SYSTIMER, systimer_.get(), 0x1000, true);
    attach(USB_SERIAL_JTAG, usb_jtag_.get());
    attach(INTMTX, intmtx_.get());
    attach(SHA, sha_.get());
    attach(IO_MUX, gpio_dev_->iomux());
    attach(GPIO, gpio_dev_.get());
    attach(PCR, pcr_.get());
    attach(I2C_ANA_MST, i2c_ana_.get(), 0x800);
    attach(LP_CLKRST, lp_clkrst_.get(), 0x400);
    attach(EFUSE, efuse_.get(), 0x400);
    attach(LP_TIMER, lp_timer_.get(), 0x400);
    attach(LP_AON, lp_aon_.get(), 0x400);
    attach(LP_WDT, lp_wdt_.get(), 0x400);
    attach(LPPERI, lpperi_.get(), 0x400);
    attach(INTPRI, intpri_.get());
    attach(EXTMEM, cache_.get());
    attach_region(PLIC_MX, 0x400, plic_.get());
    attach_region(CLINT_M, 0x800, clint_.get());

    RomProfile rp;
    rp.chip_name = "ESP32-C6";
    rp.banner = "ESP-ROM:esp32c6-20220919\r\nBuild:Sep 19 2022\r\n";
    rp.image_chip_id = 13;
    rp.boot_flash = 0xC;
    rp.boot_download = 0x4;
    rp.ram = {{HP_SRAM_BASE, HP_SRAM_SIZE}, {LP_SRAM_BASE, LP_SRAM_SIZE}};
    rp.cache = {{CACHE_BASE, CACHE_WINDOW}};
    rp.mmu_entries = c6::Spi0Mmu::kEntries;
    rp.rom_file = "roms/esp32c6_rev0_rom.elf";
    finish_setup(rp, ROM_BASE, ROM_SIZE);
}

Esp32c6::~Esp32c6() = default;

// MMU entry i maps one page of the 0x4200_0000 window (instruction and data
// accesses share it). A page size change remaps every entry.
void Esp32c6::remap_mmu(int index) {
    cpu_.flush_icache();
    const uint32_t ps = spi0_->page_size();
    if (index < 0 || ps != mapped_page_size_) {
        unmap(CACHE_BASE, CACHE_WINDOW);
        mapped_page_size_ = ps;
        for (int i = 0; i < c6::Spi0Mmu::kEntries; ++i) map_mmu_entry(i, ps);
        return;
    }
    map_mmu_entry(index, ps);
}

void Esp32c6::map_mmu_entry(int index, uint32_t ps) {
    const uint32_t vaddr = CACHE_BASE + uint32_t(index) * ps;
    const uint32_t value = spi0_->entry(index);
    const uint32_t page = value & 0x1FF;
    if (!(value & c6::Spi0Mmu::kValid) || uint64_t(page + 1) * ps > flash_.size()) {
        unmap(vaddr, ps);
        return;
    }
    map(vaddr, flash_.data() + page * ps, ps, false);
}

void Esp32c6::reset_peripherals(BootMode mode, uint32_t reason) {
    // The LP domain survives everything but a power-on / RTC watchdog reset.
    if (is_power_on_reset(reason)) {
        lp_aon_->reset();
        lp_timer_->reset();
        lp_clkrst_->reset();
        lp_wdt_->reset();
        i2c_ana_->reset();
    }
    uart0_->reset();
    uart1_->reset();
    gpio_dev_->reset();
    gpio_dev_->set_strap(mode == BootMode::Download ? 0x4 : 0xC);
    intmtx_->reset();
    plic_->reset();
    intpri_->reset();
    pcr_->reset();  // back to the XTAL clock
    systimer_->reset();
    timg0_->reset();
    timg1_->reset();
    spi1_->reset();
    usb_jtag_->reset();
    cache_->reset();
    lpperi_->reset();
    sha_->reset();
    clint_->reset();
    spi0_->reset();
}

// The ROM's startup code installs its UART console (ets_install_uart_printf);
// until then ets_printf output goes nowhere.
void Esp32c6::init_rom_state() {
    if (uint32_t f = rom_symbol("ets_write_char_uart")) write_rom_word("_putc1", f);
}

std::vector<int> Esp32c6::user_gpios() const {
    // GPIO24-30 belong to the SPI flash.
    std::vector<int> v;
    for (int i = 0; i <= 23; ++i) v.push_back(i);
    return v;
}

void Esp32c6::apply_clocks() { apply_cpu_clock(pcr_->cpu_hz()); }

// State the real ROM's startup code leaves behind: its vector table, interrupts
// globally on, threshold 1, UART0 routed to CPU line 5 (still masked).
void Esp32c6::on_cpu_start() {
    cpu_.mtvec = ROM_VECTOR_TABLE | 1;
    cpu_.mstatus |= 0x8;  // MIE
    intmtx_->write(c6::irq::UART0 * 4, 5);
    plic_->write(0x10 + 5 * 4, 1);  // PLIC_MXINT5_PRI
    plic_->write(0x90, 1);          // PLIC_MXINT_THRESH
}

}  // namespace esp32sim
