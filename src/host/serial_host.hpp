// Host-side serial endpoints: how other software on the PC reaches the
// emulated chip's UART. Each endpoint runs its own I/O thread and hands
// events to the emulation thread through a queue.
#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace esp32sim {

struct SerialEvent {
    enum class Kind { Data, Lines, Connected, Disconnected } kind = Kind::Data;
    std::vector<uint8_t> data;
    bool dtr = false, rts = false;
};

class SerialHost {
public:
    virtual ~SerialHost() = default;
    virtual bool start(std::string* error) = 0;
    virtual void stop() = 0;
    // Bytes transmitted by the chip, to be delivered to the connected client.
    virtual void write(const uint8_t* data, size_t len) = 0;
    virtual std::string describe() const = 0;

    bool poll(SerialEvent& ev);

protected:
    void push(SerialEvent ev);

private:
    std::mutex mutex_;
    std::deque<SerialEvent> queue_;
};

// Raw TCP ("socket://host:port" in pyserial/esptool) or RFC 2217
// ("rfc2217://host:port"), which also carries DTR/RTS so esptool's automatic
// reset into download mode works.
std::unique_ptr<SerialHost> make_tcp_serial(const std::string& bind_addr, int port, bool rfc2217);

// POSIX pseudo-terminal exposed under a stable symlink path. Returns null with
// an error message on platforms without PTYs (Windows).
std::unique_ptr<SerialHost> make_pty_serial(const std::string& link_path, std::string* error);

// Serial device: one end of a virtual null-modem pair. On Windows a com0com
// port such as "COM20" (tools then open its partner, e.g. COM21); elsewhere a
// tty path such as /dev/tnt0 (tty0tty). The partner's DTR/RTS arrive as
// DSR/CTS and drive the auto-reset circuit.
std::unique_ptr<SerialHost> make_com_serial(const std::string& port);

}  // namespace esp32sim
