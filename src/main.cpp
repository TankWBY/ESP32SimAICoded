// esp32sim: an ESP32-family hardware emulator that shows up as a serial port.
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "boards/boards.hpp"
#include "host/serial_host.hpp"
#include "host/ui_server.hpp"
#include "soc/chip.hpp"
#include "util/log.hpp"

using namespace esp32sim;

namespace {

std::atomic<bool> g_quit{false};

void on_signal(int) { g_quit = true; }

struct Options {
    std::string board = "esp32-c3-devkitm-1";
    std::string chip;           // override board mcu
    std::string flash_path;     // default derived from board
    std::string flash_size;
    std::string boards_db;      // empty = search next to the executable, then the source tree
    std::string pio_dir;        // raw PlatformIO manifests
    std::string bind = "127.0.0.1";
    std::vector<std::string> ptys;
    std::vector<std::string> coms;
    std::vector<int> tcp_ports, rfc_ports;
    BootMode boot = BootMode::Flash;
    bool echo = false;
    bool gpio_log = true;
    bool usb_console = false;
    bool console = true;
    bool list_boards = false, list_chips = false;
    std::string list_filter;
    std::string mac;
    std::string rom;
    bool no_rom = false;
    std::vector<std::string> elfs;
    double speed = 1.0;  // 0 = unthrottled
    double bench = 0;    // emulated seconds to run flat out, then exit
    int ui_port = 8080;  // 0 = no web UI
    bool open_ui = false;
};

void usage() {
    std::printf(
        "esp32sim %s - ESP32 hardware emulator with a virtual serial port\n"
        "\n"
        "usage: esp32sim [options]\n"
        "\n"
        "Board / chip:\n"
        "  --board ID           PlatformIO board id (default esp32-c3-devkitm-1)\n"
        "  --chip MCU           force a chip variant (esp32c3, esp32c6, ...)\n"
        "  --list-boards [TXT]  list known boards, optionally filtered\n"
        "  --list-chips         list chip variants and their emulation status\n"
        "  --boards-db FILE     board cache (default: bundled boards/boards.json)\n"
        "  --pio-boards [DIR]   read boards straight from a PlatformIO install\n"
        "                       (default ~/.platformio/platforms/espressif32/boards)\n"
        "\n"
        "Flash:\n"
        "  --flash FILE         flash image file, created if missing (default flash-<board>.bin)\n"
        "  --flash-size SIZE    e.g. 4MB (default: from board)\n"
        "  --mac AA:BB:CC:DD:EE:FF  base MAC address burned into eFuse\n"
        "  --elf FILE           firmware ELF for symbol names in crash reports (repeatable)\n"
        "  --rom FILE           Espressif mask-ROM ELF (tools/fetch_roms.py); needed for ESP-IDF/Arduino apps\n"
        "                       (default: roms/<chip>_revN_rom.elf next to the executable, if present)\n"
        "  --no-rom             ignore any ROM image: built-in ROM emulation only (bare-metal firmware)\n"
        "\n"
        "Serial endpoints (any combination; default --rfc2217 5555):\n"
        "  --rfc2217 PORT       RFC 2217 server: rfc2217://localhost:PORT (DTR/RTS reset works)\n"
        "  --tcp PORT           raw TCP: socket://localhost:PORT\n"
        "  --pty PATH           pseudo-terminal symlinked at PATH (Linux/macOS)\n"
        "  --com PORT           open a serial device: one end of a virtual COM pair, e.g. COM20 with\n"
        "                       com0com on Windows (tools use the partner port), /dev/tnt0 with tty0tty\n"
        "  --bind ADDR          listen address for TCP endpoints and the UI (default 127.0.0.1)\n"
        "\n"
        "Web UI (board view, pins, serial console):\n"
        "  --ui PORT            serve the UI on http://localhost:PORT (default 8080)\n"
        "  --no-ui              no web UI\n"
        "  --open               open the UI in the default browser\n"
        "\n"
        "Runtime:\n"
        "  --boot flash|download  strap state at power-on (default flash)\n"
        "  --echo               also print chip serial output to stdout\n"
        "  --no-gpio-log        do not log GPIO output changes\n"
        "  --usb-console        serial endpoints show the USB-Serial-JTAG console instead of UART0 output\n"
        "  --speed X            emulated speed relative to real time (default 1, 0 = max)\n"
        "  --no-console         do not read control commands from stdin\n"
        "  --bench SECONDS      run SECONDS of emulated time as fast as possible, report speed, exit\n"
        "  -v, -vv              more logging\n"
        "  --version, --help\n"
        "\n"
        "Console commands (type + Enter): r = reset & run, d = reset into download mode,\n"
        "  pins = show GPIO states, p <gpio> 0|1|z = drive a pin from outside (z releases), q = quit\n",
        ESP32SIM_VERSION);
}

bool parse_mac(const std::string& s, std::array<uint8_t, 6>& out) {
    unsigned v[6];
    if (std::sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
    for (int i = 0; i < 6; ++i) out[size_t(i)] = uint8_t(v[i]);
    return true;
}

bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s needs %s\n", a.c_str(), what);
                std::exit(2);
            }
            return argv[++i];
        };
        auto optional_next = [&]() -> std::string {
            if (i + 1 < argc && argv[i + 1][0] != '-') return argv[++i];
            return {};
        };
        if (a == "-h" || a == "--help") { usage(); std::exit(0); }
        else if (a == "--version") { std::printf("esp32sim %s\n", ESP32SIM_VERSION); std::exit(0); }
        else if (a == "--board") o.board = next("a board id");
        else if (a == "--chip") o.chip = next("a chip name");
        else if (a == "--flash") o.flash_path = next("a file");
        else if (a == "--flash-size") o.flash_size = next("a size");
        else if (a == "--boards-db") o.boards_db = next("a file");
        else if (a == "--pio-boards") { o.pio_dir = optional_next(); if (o.pio_dir.empty()) o.pio_dir = BoardDb::default_platformio_dir(); }
        else if (a == "--list-boards") { o.list_boards = true; o.list_filter = optional_next(); }
        else if (a == "--list-chips") o.list_chips = true;
        else if (a == "--rfc2217") o.rfc_ports.push_back(std::atoi(next("a port").c_str()));
        else if (a == "--tcp") o.tcp_ports.push_back(std::atoi(next("a port").c_str()));
        else if (a == "--pty") o.ptys.push_back(next("a path"));
        else if (a == "--com") o.coms.push_back(next("a port name"));
        else if (a == "--bind") o.bind = next("an address");
        else if (a == "--ui") o.ui_port = std::atoi(next("a port").c_str());
        else if (a == "--no-ui") o.ui_port = 0;
        else if (a == "--open") o.open_ui = true;
        else if (a == "--echo") o.echo = true;
        else if (a == "--no-gpio-log") o.gpio_log = false;
        else if (a == "--usb-console") o.usb_console = true;
        else if (a == "--no-console") o.console = false;
        else if (a == "--mac") o.mac = next("a MAC address");
        else if (a == "--rom") o.rom = next("a ROM ELF file");
        else if (a == "--no-rom") o.no_rom = true;
        else if (a == "--elf") o.elfs.push_back(next("an ELF file"));
        else if (a == "--speed") o.speed = std::atof(next("a factor").c_str());
        else if (a == "--bench") o.bench = std::atof(next("seconds").c_str());
        else if (a == "--boot") {
            std::string m = next("flash|download");
            if (m == "flash") o.boot = BootMode::Flash;
            else if (m == "download") o.boot = BootMode::Download;
            else { std::fprintf(stderr, "error: --boot expects flash or download\n"); return false; }
        }
        else if (a == "-v") set_log_level(LogLevel::Debug);
        else if (a == "-vv") set_log_level(LogLevel::Trace);
        else {
            std::fprintf(stderr, "error: unknown option %s (see --help)\n", a.c_str());
            return false;
        }
    }
    return true;
}

std::string lower(std::string s) {
    for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

void list_chips() {
    std::printf("%-10s %-10s %-24s %s\n", "MCU", "NAME", "CORE", "STATUS");
    for (const auto& c : chip_catalog())
        std::printf("%-10s %-10s %-24s %s - %s\n", c.mcu.c_str(), c.name.c_str(), c.core.c_str(),
                    c.supported ? "emulated" : "not yet", c.note.c_str());
}

void list_boards(const BoardDb& db, const std::string& filter) {
    std::printf("Boards from %s\n\n", db.source().c_str());
    std::printf("%-34s %-9s %-6s %-6s %s\n", "ID", "MCU", "FLASH", "EMU", "NAME");
    size_t n = 0;
    std::string f = lower(filter);
    for (const auto& b : db.boards()) {
        if (!f.empty() && lower(b.id + " " + b.name + " " + b.mcu).find(f) == std::string::npos) continue;
        bool emu = false;
        for (const auto& c : chip_catalog()) if (c.mcu == b.mcu) emu = c.supported;
        std::printf("%-34s %-9s %-6s %-6s %s\n", b.id.c_str(), b.mcu.c_str(),
                    b.flash_size ? (std::to_string(b.flash_size >> 20) + "MB").c_str() : "?", emu ? "yes" : "no",
                    b.name.c_str());
        ++n;
    }
    std::printf("\n%zu boards\n", n);
}

// Reads simple control commands from stdin on a background thread.
class Console {
public:
    void start() {
        thread_ = std::thread([this] {
            std::string line;
            while (!g_quit && std::getline(std::cin, line)) {
                std::lock_guard<std::mutex> lock(m_);
                cmds_.push_back(line);
            }
        });
        thread_.detach();
    }
    bool poll(std::string& cmd) {
        std::lock_guard<std::mutex> lock(m_);
        if (cmds_.empty()) return false;
        cmd = cmds_.front();
        cmds_.erase(cmds_.begin());
        return true;
    }

private:
    std::thread thread_;
    std::mutex m_;
    std::vector<std::string> cmds_;
};

void print_pins(const Chip& chip) {
    std::printf("GPIO  FUNCTION          DIR     PULL  CHIP EXT  LEVEL IRQ\n");
    for (int g : chip.user_gpios()) {
        PinInfo p = chip.pin_info(g);
        auto lv = [](int v) { return v < 0 ? "-" : v ? "1" : "0"; };
        std::printf("%-5d %-17s %-7s %-5s %-4s %-4s %-5s %s\n", g, p.function.c_str(),
                    p.output ? (p.input ? "in/out" : "out") : (p.input ? "in" : "-"),
                    p.pull_up ? "up" : p.pull_down ? "down" : "-", lv(p.chip_level), lv(p.external),
                    p.level < 0 ? "float" : lv(p.level), p.interrupt.c_str());
    }
    std::fflush(stdout);
}

}  // namespace


namespace {

// The board currently being emulated.
struct ActiveBoard {
    std::string id, name, mcu;
    std::string flash_path;
};

std::string catalog_json(const BoardDb& db) {
    std::string j = "{\"families\":[";
    bool first = true;
    for (const auto& c : chip_catalog()) {
        if (!first) j += ',';
        first = false;
        j += "{\"mcu\":\"" + json_escape(c.mcu) + "\",\"name\":\"" + json_escape(c.name) + "\",\"core\":\"" +
             json_escape(c.core) + "\",\"supported\":" + (c.supported ? "true" : "false") + ",\"note\":\"" +
             json_escape(c.note) + "\"}";
    }
    j += "],\"boards\":[";
    first = true;
    for (const auto& b : db.boards()) {
        if (!first) j += ',';
        first = false;
        j += "{\"id\":\"" + json_escape(b.id) + "\",\"name\":\"" + json_escape(b.name) + "\",\"mcu\":\"" +
             json_escape(b.mcu) + "\",\"vendor\":\"" + json_escape(b.vendor) + "\",\"url\":\"" + json_escape(b.url) +
             "\",\"flash\":" + std::to_string(b.flash_size) +
             ",\"layout\":" + (find_ui_asset("layouts/" + b.id + ".json") ? "true" : "false") + "}";
    }
    return j + "]}";
}

std::string state_json(const Chip* chip, const ActiveBoard& board, bool held, const std::string& error,
                       const std::vector<std::string>& endpoints) {
    std::string j = "{\"board\":\"" + json_escape(board.id) + "\",\"board_name\":\"" + json_escape(board.name) +
                    "\",\"mcu\":\"" + json_escape(board.mcu) + "\",\"flash\":\"" + json_escape(board.flash_path) +
                    "\",\"chip\":\"" + json_escape(chip ? chip->name() : "") + "\",\"status\":\"" +
                    json_escape(chip ? chip->status() : "") + "\",\"held\":" +
                    (held ? "true" : "false") + ",\"error\":\"" + json_escape(error) + "\",\"serial\":[";
    for (size_t i = 0; i < endpoints.size(); ++i) j += (i ? ",\"" : "\"") + json_escape(endpoints[i]) + "\"";
    j += "],\"pins\":[";
    if (chip) {
        bool first = true;
        for (int g : chip->user_gpios()) {
            PinInfo p = chip->pin_info(g);
            if (!first) j += ',';
            first = false;
            j += "{\"gpio\":" + std::to_string(g) + ",\"fn\":\"" + json_escape(p.function) +
                 "\",\"out\":" + (p.output ? "1" : "0") + ",\"in\":" + (p.input ? "1" : "0") +
                 ",\"pu\":" + (p.pull_up ? "1" : "0") + ",\"pd\":" + (p.pull_down ? "1" : "0") +
                 ",\"od\":" + (p.open_drain ? "1" : "0") + ",\"drv\":" + std::to_string(p.chip_level) +
                 ",\"ext\":" + std::to_string(p.external) + ",\"lvl\":" + std::to_string(p.level) +
                 ",\"irq\":\"" + json_escape(p.interrupt) + "\"}";
        }
    }
    return j + "]}";
}

void open_browser(const std::string& url) {
#ifdef _WIN32
    std::string cmd = "start \"\" \"" + url + "\"";
#elif defined(__APPLE__)
    std::string cmd = "open '" + url + "' >/dev/null 2>&1 &";
#else
    std::string cmd = "xdg-open '" + url + "' >/dev/null 2>&1 &";
#endif
    if (std::system(cmd.c_str()) != 0) LOGW("could not open a browser; go to %s", url.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) return 2;

    if (opt.list_chips) {
        list_chips();
        return 0;
    }

    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path exe_dir = fs::absolute(fs::path(argv[0]), ec).parent_path();

    BoardDb db;
    std::string err;
    if (opt.boards_db.empty()) {
        for (const fs::path& p : {exe_dir / "boards.json", exe_dir / "boards" / "boards.json",
                                  exe_dir.parent_path() / "boards" / "boards.json", fs::path(ESP32SIM_DEFAULT_BOARDS)}) {
            if (fs::exists(p, ec)) {
                opt.boards_db = p.string();
                break;
            }
        }
    }
    bool have_db = !opt.pio_dir.empty() ? db.load_platformio_dir(opt.pio_dir, &err)
                                        : (!opt.boards_db.empty() && db.load_cache(opt.boards_db, &err));
    if (!have_db) LOGW("board list unavailable%s%s", err.empty() ? "" : ": ", err.c_str());

    if (opt.list_boards) {
        if (!have_db) return 1;
        list_boards(db, opt.list_filter);
        return 0;
    }

    std::array<uint8_t, 6> mac = ChipConfig().mac;
    if (!opt.mac.empty() && !parse_mac(opt.mac, mac)) {
        LOGE("bad --mac value");
        return 1;
    }

    // Builds the chip for a board. --chip, --flash and --flash-size only apply
    // to the board given on the command line; boards picked later in the UI
    // use their own defaults and their own flash-<board>.bin.
    auto make_chip = [&](const std::string& board_id, bool initial, ActiveBoard& active,
                         std::string* error) -> std::unique_ptr<Chip> {
        ChipConfig cfg;
        cfg.mac = mac;
        const BoardInfo* board = db.find(board_id);
        if (board) {
            cfg.variant = board->mcu;
            if (board->flash_size) cfg.flash_size = board->flash_size;
            if (board->f_cpu) cfg.cpu_hz = board->f_cpu;
            LOGI("board: %s (%s, %s, %u MB flash)", board->id.c_str(), board->name.c_str(), board->mcu.c_str(),
                 cfg.flash_size >> 20);
        } else if (!initial || (opt.chip.empty() && (have_db || board_id != Options().board))) {
            *error = "unknown board '" + board_id + "' (try --list-boards, or give --chip)";
            return nullptr;
        } else {
            LOGI("board: generic %s (no board list), %u MB flash",
                 opt.chip.empty() ? cfg.variant.c_str() : opt.chip.c_str(), cfg.flash_size >> 20);
        }
        if (initial && !opt.chip.empty()) cfg.variant = opt.chip;
        if (initial && !opt.flash_size.empty()) cfg.flash_size = BoardDb::parse_size(opt.flash_size);
        if (cfg.flash_size < 0x100000 || (cfg.flash_size & 0xFFFF)) {
            *error = "flash size must be a multiple of 64 KiB and at least 1 MB";
            return nullptr;
        }
        cfg.trace_gpio = opt.gpio_log;
        cfg.usb_console = opt.usb_console;
        cfg.rom_path = opt.rom;
        if (cfg.rom_path.empty() && !opt.no_rom) {
            std::string name = cfg.variant + (cfg.variant == "esp32c6" ? "_rev0_rom.elf" : "_rev3_rom.elf");
            for (const fs::path& p :
                 {exe_dir / "roms" / name, exe_dir.parent_path() / "roms" / name, fs::path("roms") / name}) {
                if (fs::exists(p, ec)) {
                    cfg.rom_path = p.string();
                    break;
                }
            }
            if (cfg.rom_path.empty())
                LOGW("no ROM image found: ESP-IDF/Arduino firmware needs one (run tools/fetch_roms.py)");
        }
        cfg.flash_path = initial && !opt.flash_path.empty() ? opt.flash_path
                                                            : "flash-" + (board ? board->id : cfg.variant) + ".bin";
        auto chip = create_chip(cfg, error);
        if (!chip) return nullptr;
        for (const auto& e : opt.elfs) chip->add_symbols(e);
        active.id = board ? board->id : board_id;
        active.name = board ? board->name : "Generic " + chip->name();
        active.mcu = cfg.variant;
        active.flash_path = cfg.flash_path;
        LOGI("chip: %s @ %u MHz, flash %s", chip->name().c_str(), chip->cpu_hz() / 1000000, cfg.flash_path.c_str());
        return chip;
    };

    ActiveBoard active;
    std::unique_ptr<Chip> chip = make_chip(opt.board, true, active, &err);
    if (!chip) {
        LOGE("%s", err.c_str());
        return 1;
    }

    // Serial endpoints.
    if (opt.rfc_ports.empty() && opt.tcp_ports.empty() && opt.ptys.empty() && opt.coms.empty()) opt.rfc_ports.push_back(5555);
    std::vector<std::unique_ptr<SerialHost>> hosts;
    for (int p : opt.rfc_ports) hosts.push_back(make_tcp_serial(opt.bind, p, true));
    for (int p : opt.tcp_ports) hosts.push_back(make_tcp_serial(opt.bind, p, false));
    for (const auto& path : opt.ptys) {
        auto h = make_pty_serial(path, &err);
        if (!h) { LOGE("%s", err.c_str()); return 1; }
        hosts.push_back(std::move(h));
    }
    for (const auto& port : opt.coms) hosts.push_back(make_com_serial(port));
    std::vector<std::string> endpoints;
    for (auto& h : hosts) {
        if (!h->start(&err)) {
            LOGE("%s", err.c_str());
            return 1;
        }
        endpoints.push_back(h->describe());
        LOGI("serial: %s", h->describe().c_str());
    }

    std::unique_ptr<UiServer> ui;
    if (opt.ui_port > 0 && opt.bench <= 0) {
        ui = std::make_unique<UiServer>(opt.bind, opt.ui_port);
        if (ui->start(&err)) {
            ui->set_catalog(catalog_json(db));
            LOGI("ui: %s", ui->url().c_str());
            if (opt.open_ui) open_browser(ui->url());
        } else {
            LOGW("ui: %s; continuing without the web UI (use --ui PORT)", err.c_str());
            ui.reset();
        }
    }

    std::mutex out_mutex;
    auto attach = [&](Chip& c) {
        c.serial_output = [&](const uint8_t* d, size_t n) {
            for (auto& h : hosts) h->write(d, n);
            if (ui) ui->publish_serial(d, n);
            if (opt.echo) {
                std::lock_guard<std::mutex> lock(out_mutex);
                std::fwrite(d, 1, n, stdout);
                std::fflush(stdout);
            }
        };
    };
    attach(*chip);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
#ifdef SIGPIPE
    std::signal(SIGPIPE, SIG_IGN);  // a client vanishing mid-send must not kill us
#endif

    Console console;
    if (opt.console) console.start();

    chip->reset(opt.boot);

    if (opt.bench > 0) {
        auto b0 = std::chrono::steady_clock::now();
        const uint64_t total = uint64_t(opt.bench * chip->cpu_hz());
        for (uint64_t done = 0; done < total && !g_quit; done += chip->cpu_hz() / 1000) chip->run_for(chip->cpu_hz() / 1000);
        double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - b0).count();
        std::printf("\nbench: %.2f s emulated in %.2f s wall (%.0f%% of real time), %.1f MIPS\n", opt.bench, wall,
                    opt.bench / wall * 100, double(chip->instructions()) / wall / 1e6);
        for (auto& h : hosts) h->stop();
        chip->shutdown();
        return 0;
    }

    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    auto now_s = [&] { return std::chrono::duration<double>(clock::now() - t0).count(); };
    AutoResetCircuit circuit;
    double emu_time = 0;
    double hz = double(chip->cpu_hz()) * (opt.speed > 0 ? opt.speed : 1.0);
    bool ui_hold = false;      // RESET button held in the UI
    std::string ui_error;      // last failed UI request, shown in the UI
    double state_at = 0;

    double stats_at = 1, stats_wall = 0;
    uint64_t stats_insn = 0;
    while (!g_quit) {
        double now = now_s();
        if (now >= stats_at) {
            uint64_t insn = chip->instructions();
            double mips = double(insn - stats_insn) / (now - stats_wall) / 1e6;
            if (insn != stats_insn)
                LOGD("perf: %.1f MIPS (%.0f%% of a %u MHz core) %s", mips, mips * 1e6 / chip->cpu_hz() * 100,
                     chip->cpu_hz() / 1000000, chip->where().c_str());
            stats_insn = insn;
            stats_wall = now;
            stats_at = now + (log_level() >= LogLevel::Trace ? 1 : 5);
        }

        for (auto& h : hosts) {
            SerialEvent ev;
            while (h->poll(ev)) {
                switch (ev.kind) {
                case SerialEvent::Kind::Data:
                    chip->serial_input(ev.data.data(), ev.data.size());
                    break;
                case SerialEvent::Kind::Lines:
                    LOGD("lines: DTR=%d RTS=%d", ev.dtr, ev.rts);
                    circuit.set_lines(ev.dtr, ev.rts, now);
                    if (circuit.in_reset()) chip->hold_in_reset();
                    break;
                default:
                    break;
                }
            }
        }
        BootMode mode;
        if (circuit.poll(now, &mode)) {
            LOGI("reset released by DTR/RTS -> %s boot", mode == BootMode::Download ? "download" : "flash");
            chip->reset(mode);
        }

        std::string cmd;
        while (console.poll(cmd)) {
            if (cmd == "r") chip->reset(BootMode::Flash);
            else if (cmd == "d") chip->reset(BootMode::Download);
            else if (cmd == "q") g_quit = true;
            else if (cmd == "pins") print_pins(*chip);
            else if (cmd.size() > 2 && cmd[0] == 'p' && cmd[1] == ' ') {
                int gpio = -1;
                char lv[8] = {};
                if (std::sscanf(cmd.c_str() + 2, "%d %7s", &gpio, lv) == 2 && (lv[0] == '0' || lv[0] == '1' || lv[0] == 'z'))
                    chip->set_pin_input(gpio, lv[0] == 'z' ? -1 : lv[0] - '0');
                else LOGW("console: p <gpio> 0|1|z");
            }
            else if (!cmd.empty()) LOGW("console: unknown command '%s' (r, d, q, pins, p <gpio> 0|1|z)", cmd.c_str());
        }

        if (ui) {
            UiCommand uc;
            while (ui->poll(uc)) {
                if (uc.name == "pin") {
                    const Json& lv = uc.args["level"];
                    chip->set_pin_input(int(uc.args["gpio"].num(-1)), lv.type() == Json::Type::Number ? int(lv.num()) : -1);
                } else if (uc.name == "reset") {
                    // The RESET button: EN low while pressed, boot on release.
                    bool hold = uc.args["hold"].boolean(false);
                    std::string m = uc.args["mode"].str();
                    if (hold) {
                        ui_hold = true;
                        chip->hold_in_reset();
                    } else {
                        ui_hold = false;
                        chip->reset(m == "download" ? BootMode::Download : BootMode::Flash);
                    }
                } else if (uc.name == "serial") {
                    std::string text = uc.args["text"].str();
                    chip->serial_input(reinterpret_cast<const uint8_t*>(text.data()), text.size());
                } else if (uc.name == "board") {
                    std::string id = uc.args["board"].str();
                    if (id.empty() || id == active.id) continue;
                    LOGI("ui: switching to board %s", id.c_str());
                    chip->shutdown();
                    ActiveBoard next;
                    std::string e;
                    auto fresh = make_chip(id, false, next, &e);
                    if (!fresh) {
                        LOGW("ui: %s", e.c_str());
                        ui_error = e;
                        fresh = make_chip(active.id, active.id == opt.board, next, &e);  // back to the old board
                        if (!fresh) {
                            LOGE("%s", e.c_str());
                            g_quit = true;
                            break;
                        }
                    } else {
                        ui_error.clear();
                    }
                    chip = std::move(fresh);
                    active = next;
                    attach(*chip);
                    hz = double(chip->cpu_hz()) * (opt.speed > 0 ? opt.speed : 1.0);
                    chip->reset(BootMode::Flash);
                    const char banner[] = "\r\n--- esp32sim: board changed ---\r\n";
                    ui->publish_serial(reinterpret_cast<const uint8_t*>(banner), sizeof banner - 1);
                } else {
                    LOGW("ui: unknown request %s", uc.name.c_str());
                }
            }
            if (now >= state_at) {
                ui->publish_state(state_json(chip.get(), active, ui_hold || circuit.in_reset(), ui_error, endpoints));
                state_at = now + 0.05;
            }
        }

        if (opt.speed <= 0 && !chip->idle()) {
            chip->run_for(uint64_t(hz / 1000));  // as fast as possible, 1 ms slices
            emu_time = now;
            continue;
        }
        if (emu_time < now) {
            double dt = now - emu_time;
            if (dt > 0.1) {  // can't keep up (or we were suspended): drop the backlog
                emu_time = now - 0.002;
                dt = 0.002;
            }
            if (dt > 0.002) dt = 0.002;
            chip->run_for(uint64_t(dt * hz));
            emu_time += dt;
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(chip->idle() ? 1000 : 200));
        }
    }

    LOGI("shutting down");
    if (ui) ui->stop();
    for (auto& h : hosts) h->stop();
    chip->shutdown();
    return 0;
}
