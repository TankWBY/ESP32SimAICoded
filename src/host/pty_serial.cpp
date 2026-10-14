// POSIX pseudo-terminal endpoint. Programs that want a device path
// (serial monitors, IDEs, `idf.py monitor -p`, PlatformIO) can open the
// symlink like a normal serial port.
#include "host/serial_host.hpp"
#include "util/log.hpp"

#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif
#endif

namespace esp32sim {

#ifndef _WIN32
namespace {

class PtySerial : public SerialHost {
public:
    explicit PtySerial(std::string link) : link_(std::move(link)) {}
    ~PtySerial() override { stop(); }

    bool start(std::string* error) override {
        char name[256] = {};
        if (openpty(&master_, &slave_, name, nullptr, nullptr) != 0) {
            *error = "openpty() failed";
            return false;
        }
        slave_name_ = name;
        termios t{};
        tcgetattr(slave_, &t);
        cfmakeraw(&t);
        tcsetattr(slave_, TCSANOW, &t);
        fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK);
        if (!link_.empty()) {
            ::unlink(link_.c_str());
            if (::symlink(slave_name_.c_str(), link_.c_str()) != 0) {
                *error = "cannot create symlink " + link_ + " -> " + slave_name_;
                return false;
            }
        }
        running_ = true;
        thread_ = std::thread([this] { loop(); });
        return true;
    }

    void stop() override {
        if (!running_) return;
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (!link_.empty()) ::unlink(link_.c_str());
        ::close(master_);
        ::close(slave_);
    }

    void write(const uint8_t* data, size_t len) override {
        while (len) {
            ssize_t n = ::write(master_, data, len);
            if (n <= 0) return;  // nobody reading and buffer full: drop, like a real UART would
            data += n;
            len -= size_t(n);
        }
    }

    std::string describe() const override { return (link_.empty() ? slave_name_ : link_ + " -> " + slave_name_); }

private:
    void loop() {
        uint8_t buf[4096];
        while (running_) {
            pollfd p{master_, POLLIN, 0};
            int r = ::poll(&p, 1, 100);
            if (r <= 0) continue;
            ssize_t n = ::read(master_, buf, sizeof buf);
            if (n > 0) push({SerialEvent::Kind::Data, std::vector<uint8_t>(buf, buf + n), false, false});
            else if (n < 0 && errno != EAGAIN && errno != EIO) break;
            else if (n <= 0) ::usleep(10000);
        }
    }

    std::string link_, slave_name_;
    int master_ = -1, slave_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace

std::unique_ptr<SerialHost> make_pty_serial(const std::string& link_path, std::string* error) {
    (void)error;
    return std::make_unique<PtySerial>(link_path);
}
#else
std::unique_ptr<SerialHost> make_pty_serial(const std::string&, std::string* error) {
    *error = "pseudo-terminals are not available on Windows; use --rfc2217 or --tcp "
             "(see README for exposing a COM port with com0com)";
    return nullptr;
}
#endif

}  // namespace esp32sim
