#include "soc/esp32c3/esp32c3.hpp"

#include "util/log.hpp"

namespace esp32sim {

using c3::kSigU0Rxd, c3::kSigU1Txd, c3::kSigU1Rxd, c3::kSigFspiq, c3::kSigFspid;
using namespace c3map;

Esp32c3::Esp32c3(const ChipConfig& cfg)
    : RiscvSoc(cfg, PERIPH_BASE, PERIPH_SIZE),
      sram_(IRAM_SIZE, 0),
      rtc_fast_(kMmuPage, 0),
      rom_image_(ROM_SIZE, 0),
      drom_image_(DROM_MASK_SIZE, 0) {
    map(IRAM_BASE, sram_.data(), IRAM_SIZE, true);
    map(DRAM_BASE, sram_.data() + (SRAM1_IRAM - IRAM_BASE), DRAM_SIZE, true);
    map(RTC_FAST_BASE, rtc_fast_.data(), kMmuPage, true);
    load_rom(ROM_BASE, rom_image_);
    load_rom(DROM_MASK_BASE, drom_image_);
    map(ROM_BASE, rom_image_.data(), ROM_SIZE, false);
    map(DROM_MASK_BASE, drom_image_.data(), DROM_MASK_SIZE, false);

    uart0_ = std::make_unique<c3::Uart>("UART0");
    uart1_ = std::make_unique<c3::Uart>("UART1");
    gpio_dev_ = std::make_unique<c3::Gpio>("GPIO", c3::kEsp32c3Pads);
    gpio_dev_->trace = cfg.trace_gpio;
    gpio_ = gpio_dev_.get();
    systimer_ = std::make_unique<c3::Systimer>("SYSTIMER", clocks_);
    timg0_ = std::make_unique<c3::Timg>("TIMG0", clocks_, c3::irq::TG0_T0, c3::irq::TG0_WDT);
    timg1_ = std::make_unique<c3::Timg>("TIMG1", clocks_, c3::irq::TG1_T0, c3::irq::TG1_WDT);
    system_ = std::make_unique<c3::SystemReg>("SYSTEM");
    intmatrix_ = std::make_unique<c3::IntMatrix>("INTMATRIX", c3::irq::kSources, true);
    timed_ = {systimer_.get(), timg0_.get(), timg1_.get()};
    efuse_ = std::make_unique<c3::Efuse>("EFUSE", cfg.mac, 4u << 18);  // wafer minor 4: chip revision v0.4
    rtc_cntl_ = std::make_unique<c3::RtcCntl>("RTC_CNTL");
    mmu_ = std::make_unique<c3::Mmu>("MMU");
    usb_jtag_ = std::make_unique<c3::UsbSerialJtag>("USB_SERIAL_JTAG");
    extmem_ = std::make_unique<c3::Extmem>("EXTMEM");
    syscon_ = std::make_unique<c3::Syscon>("SYSCON", 0xB0);
    sha_ = std::make_unique<c3::Sha>("SHA");
    spi1_ = std::make_unique<SpiMemController>("SPI1", flash_);
    adc_ = std::make_unique<c3::SarAdc>("APB_SARADC");
    aes_ = std::make_unique<c3::Aes>("AES");
    i2c0_ = std::make_unique<I2cController>("I2C0");
    spi2_ = std::make_unique<c3::GpSpi>("SPI2");
    attach_i2c_devices(*i2c0_);
    ledc_ = std::make_unique<c3::Ledc>("LEDC", true);
    gdma_ = std::make_unique<c3::Gdma>("GDMA", *this, 0x3FC00000, c3::Gdma::Layout::C3);
    // ADC1 channels 0-4 are GPIO0-4, ADC2 channel 0 is GPIO5. A pad reads
    // full scale when high and 0 when low or floating.
    adc_->sample = [this](int unit, int ch) -> uint32_t {
        int pin = unit == 0 ? (ch <= 4 ? ch : -1) : (ch == 0 ? 5 : -1);
        return pin >= 0 && gpio_dev_->info(pin).level == 1 ? 4095 : 0;
    };

    uart0_->on_tx = [this](uint8_t b) { if (!cfg_.usb_console) tx_buf_.push_back(b); };
    // UART1 TX reaches a UART's RX when the GPIO matrix routes both to one pad.
    uart1_->on_tx = [this](uint8_t b) {
        bool heard = false;
        for (auto [uart, rx_sig] : {std::pair{uart0_.get(), kSigU0Rxd}, std::pair{uart1_.get(), kSigU1Rxd}}) {
            if (gpio_dev_->signal_routed(kSigU1Txd, rx_sig) && uart->rx_space()) uart->push_rx(b), heard = true;
        }
        if (!heard) LOGD("uart1 tx: 0x%02x", b);
    };
    usb_jtag_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::USB_SERIAL_JTAG, l); };
    usb_jtag_->frame_number = [this] { return uint32_t(cpu_.cycles * 1000 / clocks_.master_hz); };
    usb_jtag_->on_tx = [this](uint8_t b) { if (cfg_.usb_console) tx_buf_.push_back(b); };
    rtc_cntl_->on_system_reset = [this](double delay) { request_reset(BootMode::Flash, delay); };
    rtc_cntl_->slow_ticks = [this] {
        return uint64_t(double(cpu_.cycles) * c3::RtcCntl::kSlowClockHz / clocks_.master_hz);
    };
    rtc_cntl_->on_wdt = [this](double delay) { arm_rtc_wdt(delay); };
    mmu_->on_change = [this](int i, uint32_t v) { on_mmu_change(i, v); };
    auto irq = [this](int src, bool level) { intmatrix_->set_source(src, level); };
    systimer_->irq = [this](int unit, bool level) { intmatrix_->set_source(c3::irq::SYSTIMER_TARGET0 + unit, level); };
    gpio_dev_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::GPIO, l); };
    uart0_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::UART0, l); };
    uart1_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::UART1, l); };
    timg0_->irq = irq;
    gdma_->irq = [this](int ch, bool l) { intmatrix_->set_source(c3::irq::DMA_CH0 + ch, l); };
    // MISO hears MOSI when the GPIO matrix routes FSPID back into FSPIQ; an
    // unconnected MISO reads 1s.
    spi2_->exchange = [this](uint8_t mosi) -> uint8_t {
        return gpio_dev_->signal_routed(kSigFspid, kSigFspiq) ? mosi : 0xFF;
    };
    spi2_->dma_read = [this](uint8_t* d, size_t n) { return gdma_->out_read(c3::Gdma::SPI2, d, n); };
    spi2_->dma_write = [this](const uint8_t* s, size_t n) { gdma_->in_write(c3::Gdma::SPI2, s, n, true); };
    spi2_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::SPI2, l); };
    i2c0_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::I2C_EXT0, l); };
    ledc_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::LEDC, l); };
    aes_->irq = [this](bool l) { intmatrix_->set_source(c3::irq::AES, l); };
    aes_->dma_read = [this](uint8_t* d, size_t n) { return gdma_->out_read(c3::Gdma::AES, d, n); };
    aes_->dma_write = [this](const uint8_t* s, size_t n) { gdma_->in_write(c3::Gdma::AES, s, n, true); };
    sha_->dma_read = [this](uint8_t* block) { return gdma_->out_read(c3::Gdma::SHA, block, 64) == 64; };
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
    attach(GPIO, gpio_dev_.get());
    attach(IO_MUX, gpio_dev_->iomux());
    attach(RTC_CNTL, rtc_cntl_.get(), 0x800);
    attach(EFUSE, efuse_.get(), 0x800);
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
    attach(APB_SARADC, adc_.get());
    attach(AES, aes_.get());
    attach(LEDC, ledc_.get());
    attach(I2C0, i2c0_.get());
    attach(SPI2, spi2_.get());
    attach(GDMA, gdma_.get());

    RomProfile rp;
    rp.chip_name = "ESP32-C3";
    rp.banner = "ESP-ROM:esp32c3-api1-20210207\r\nBuild:Feb  7 2021\r\n";
    rp.image_chip_id = 5;
    rp.boot_flash = 0xC;
    rp.boot_download = 0x4;
    rp.ram = {{SRAM0_BASE, SRAM_END_IRAM - SRAM0_BASE}, {DRAM_BASE, DRAM_SIZE}, {RTC_FAST_BASE, RTC_FAST_SIZE}};
    rp.cache = {{IROM_BASE, CACHE_WINDOW}, {DROM_BASE, CACHE_WINDOW}};
    rp.mmu_entries = c3::Mmu::kEntries;
    rp.rom_file = "roms/esp32c3_rev3_rom.elf";
    finish_setup(rp, ROM_BASE, ROM_SIZE);
}

Esp32c3::~Esp32c3() = default;

void Esp32c3::on_mmu_change(int index, uint32_t value) {
    cpu_.flush_icache();
    const uint32_t off = uint32_t(index) * kMmuPage;
    uint32_t page = value & 0xFF;
    if ((value & c3::Mmu::kInvalid) || (page + 1) * kMmuPage > flash_.size()) {
        unmap(IROM_BASE + off, kMmuPage);
        unmap(DROM_BASE + off, kMmuPage);
        return;
    }
    uint8_t* host = flash_.data() + page * kMmuPage;
    map(IROM_BASE + off, host, kMmuPage, false);
    map(DROM_BASE + off, host, kMmuPage, false);
}

void Esp32c3::reset_peripherals(BootMode mode, uint32_t reason) {
    // Power-on and RTC watchdog resets clear the RTC domain; software and CPU
    // resets keep it (the STORE registers carry state across those).
    if (is_power_on_reset(reason)) rtc_cntl_->reset();
    uart0_->reset();
    uart1_->reset();
    gpio_dev_->reset();
    gpio_dev_->set_strap(mode == BootMode::Download ? 0x4 : 0xC);
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
    adc_->reset();
    aes_->reset();
    ledc_->reset();
    i2c0_->reset();
    spi2_->reset();
    gdma_->reset();
    mmu_->reset();
}

// The ROM's startup code enables its UART console (ets_install_uart_printf);
// uart_tx_one_char() drops characters until it has.
void Esp32c3::init_rom_state() { write_rom_word("g_uart_print", 1, 1); }

std::vector<int> Esp32c3::user_gpios() const {
    // GPIO11-17 belong to the SPI flash on every C3 module.
    return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 18, 19, 20, 21};
}

void Esp32c3::apply_clocks() {
    clocks_.apb_hz = system_->apb_hz();
    apply_cpu_clock(system_->cpu_hz());
}

// State the real ROM's startup code leaves behind: its vector table and
// dispatcher (ets_isr_attach handlers), interrupts globally on, threshold 1,
// and UART0 routed to CPU line 5 for the download-mode receive ISR (line
// still masked). Code loaded with MEM_END, such as esptool's flasher stub,
// relies on this.
void Esp32c3::on_cpu_start() {
    cpu_.mtvec = ROM_VECTOR_TABLE | 1;
    cpu_.mstatus |= 0x8;  // MIE
    intmatrix_->write(c3::irq::UART0 * 4, 5);
    intmatrix_->write(0x114 + 5 * 4, 1);  // CPU_INT_PRI_5
    intmatrix_->write(0x194, 1);          // CPU_INT_THRESH
}

}  // namespace esp32sim
