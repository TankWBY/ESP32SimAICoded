// Serial device endpoint: the emulator opens one end of a virtual null-modem
// pair and other software opens the other end like real hardware.
//
// Windows: one port of a com0com pair (e.g. COM20 or CNCB0); tools use the
// partner (e.g. COM21). com0com wires the partner's DTR to our DSR and its RTS
// to our CTS, so esptool's DTR/RTS reset sequence reaches the emulated
// auto-reset circuit.
//
// Linux/macOS: any tty device, typically one end of a tty0tty pair
// (/dev/tnt0 <-> /dev/tnt1) or a socat-made pty pair. Modem lines are read
// with TIOCMGET where the driver supports them.
#include "host/serial_host.hpp"
#include "util/log.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <errno.h>
#include <string.h>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace esp32sim {
namespace {

#ifdef _WIN32
class ComSerial : public SerialHost {
public:
    explicit ComSerial(std::string port) : port_(std::move(port)) {}
    ~ComSerial() override { stop(); }

    bool start(std::string* error) override {
        // "\\.\" works for every name, and is required for COM10 and above.
        std::string path = port_.rfind("\\\\.\\", 0) == 0 ? port_ : "\\\\.\\" + port_;
        h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED,
                         nullptr);
        if (h_ == INVALID_HANDLE_VALUE) {
            *error = "cannot open " + port_ + " (" + last_error() +
                     "); create a com0com pair first, see README 'Virtual COM port'";
            return false;
        }
        DCB dcb{};
        dcb.DCBlength = sizeof dcb;
        GetCommState(h_, &dcb);
        dcb.BaudRate = 115200;  // the baud rate is irrelevant on a virtual pair
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fOutxCtsFlow = dcb.fOutxDsrFlow = FALSE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fOutX = dcb.fInX = FALSE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;  // tells the partner "device present" (its DSR/DCD)
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        SetCommState(h_, &dcb);
        SetupComm(h_, 65536, 65536);
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout = MAXDWORD;  // ReadFile returns whatever is buffered
        SetCommTimeouts(h_, &to);
        SetCommMask(h_, EV_RXCHAR | EV_DSR | EV_CTS);
        PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR);
        write_event_ = CreateEventA(nullptr, TRUE, FALSE, nullptr);
        running_ = true;
        thread_ = std::thread([this] { loop(); });
        return true;
    }

    void stop() override {
        if (!running_) return;
        running_ = false;
        SetCommMask(h_, 0);  // wakes a pending WaitCommEvent
        if (thread_.joinable()) thread_.join();
        CloseHandle(write_event_);
        CloseHandle(h_);
        h_ = INVALID_HANDLE_VALUE;
    }

    void write(const uint8_t* data, size_t len) override {
        std::lock_guard<std::mutex> lock(write_mutex_);
        OVERLAPPED ov{};
        ov.hEvent = write_event_;
        ResetEvent(write_event_);
        DWORD n = 0;
        if (!WriteFile(h_, data, DWORD(len), &n, &ov)) {
            if (GetLastError() != ERROR_IO_PENDING) return;
            // Bounded wait: with nobody on the other end com0com may hold
            // data back; drop it rather than stall the emulation.
            if (WaitForSingleObject(write_event_, 200) != WAIT_OBJECT_0) {
                CancelIo(h_);
                GetOverlappedResult(h_, &ov, &n, TRUE);
                return;
            }
            GetOverlappedResult(h_, &ov, &n, FALSE);
        }
    }

    std::string describe() const override { return port_ + " (connect your tool to its com0com partner)"; }

private:
    static std::string last_error() {
        char buf[256] = {};
        FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, GetLastError(), 0, buf,
                       sizeof buf, nullptr);
        std::string s(buf);
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == '.')) s.pop_back();
        return s;
    }

    void check_lines() {
        DWORD ms = 0;
        if (!GetCommModemStatus(h_, &ms)) return;
        bool dtr = (ms & MS_DSR_ON) != 0, rts = (ms & MS_CTS_ON) != 0;
        if (first_ || dtr != dtr_ || rts != rts_) {
            dtr_ = dtr, rts_ = rts, first_ = false;
            push({SerialEvent::Kind::Lines, {}, dtr, rts});
        }
    }

    void drain() {
        uint8_t buf[4096];
        for (;;) {
            OVERLAPPED ov{};
            ov.hEvent = read_event_;
            DWORD n = 0;
            if (!ReadFile(h_, buf, sizeof buf, &n, &ov)) {
                if (GetLastError() != ERROR_IO_PENDING || !GetOverlappedResult(h_, &ov, &n, TRUE)) return;
            }
            if (n == 0) return;
            push({SerialEvent::Kind::Data, std::vector<uint8_t>(buf, buf + n), false, false});
        }
    }

    void loop() {
        read_event_ = CreateEventA(nullptr, TRUE, FALSE, nullptr);
        HANDLE wait_event = CreateEventA(nullptr, TRUE, FALSE, nullptr);
        check_lines();
        while (running_) {
            OVERLAPPED ov{};
            ov.hEvent = wait_event;
            DWORD mask = 0;
            if (!WaitCommEvent(h_, &mask, &ov)) {
                if (GetLastError() != ERROR_IO_PENDING) {
                    Sleep(10);
                    continue;
                }
                // Poll the modem lines now and then too: some drivers do not
                // report EV_DSR/EV_CTS reliably.
                while (running_ && WaitForSingleObject(wait_event, 50) == WAIT_TIMEOUT) check_lines();
                if (!running_) {
                    CancelIo(h_);
                    break;
                }
                DWORD dummy;
                GetOverlappedResult(h_, &ov, &dummy, FALSE);
            }
            check_lines();
            drain();
        }
        CloseHandle(wait_event);
        CloseHandle(read_event_);
    }

    std::string port_;
    HANDLE h_ = INVALID_HANDLE_VALUE;
    HANDLE write_event_ = nullptr, read_event_ = nullptr;
    std::mutex write_mutex_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    bool dtr_ = false, rts_ = false, first_ = true;
};

#else  // POSIX tty device

class ComSerial : public SerialHost {
public:
    explicit ComSerial(std::string port) : port_(std::move(port)) {}
    ~ComSerial() override { stop(); }

    bool start(std::string* error) override {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            *error = "cannot open " + port_ + ": " + std::strerror(errno);
            return false;
        }
        termios t{};
        tcgetattr(fd_, &t);
        cfmakeraw(&t);
        t.c_cflag |= CLOCAL | CREAD;
        cfsetispeed(&t, B115200);
        cfsetospeed(&t, B115200);
        tcsetattr(fd_, TCSANOW, &t);
        int bits = TIOCM_DTR | TIOCM_RTS;
        ioctl(fd_, TIOCMBIS, &bits);
        running_ = true;
        thread_ = std::thread([this] { loop(); });
        return true;
    }

    void stop() override {
        if (!running_) return;
        running_ = false;
        if (thread_.joinable()) thread_.join();
        ::close(fd_);
    }

    void write(const uint8_t* data, size_t len) override {
        while (len) {
            ssize_t n = ::write(fd_, data, len);
            if (n <= 0) return;  // peer not reading: drop, like a UART with nobody listening
            data += n;
            len -= size_t(n);
        }
    }

    std::string describe() const override { return port_; }

private:
    void check_lines() {
        int bits = 0;
        if (ioctl(fd_, TIOCMGET, &bits) != 0) return;  // driver without modem lines
        bool dtr = (bits & TIOCM_DSR) != 0, rts = (bits & TIOCM_CTS) != 0;
        if (first_ || dtr != dtr_ || rts != rts_) {
            dtr_ = dtr, rts_ = rts, first_ = false;
            push({SerialEvent::Kind::Lines, {}, dtr, rts});
        }
    }

    void loop() {
        uint8_t buf[4096];
        while (running_) {
            check_lines();
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, 20) <= 0) continue;
            ssize_t n = ::read(fd_, buf, sizeof buf);
            if (n > 0) push({SerialEvent::Kind::Data, std::vector<uint8_t>(buf, buf + n), false, false});
            else ::usleep(10000);
        }
    }

    std::string port_;
    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
    bool dtr_ = false, rts_ = false, first_ = true;
};
#endif

}  // namespace

std::unique_ptr<SerialHost> make_com_serial(const std::string& port) { return std::make_unique<ComSerial>(port); }

}  // namespace esp32sim
