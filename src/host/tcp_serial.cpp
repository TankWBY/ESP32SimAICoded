// TCP serial endpoint: raw byte stream or RFC 2217 (Telnet COM-PORT-OPTION).
#include <cstring>

#include "host/serial_host.hpp"
#include "util/log.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using sock_t = SOCKET;
static constexpr sock_t kBadSock = INVALID_SOCKET;
static void close_sock(sock_t s) { closesocket(s); }
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using sock_t = int;
static constexpr sock_t kBadSock = -1;
static void close_sock(sock_t s) { ::close(s); }
#endif

namespace esp32sim {

namespace {

// Telnet / RFC 2217 constants
constexpr uint8_t IAC = 255, DONT = 254, DO = 253, WONT = 252, WILL = 251, SB = 250, SE = 240;
constexpr uint8_t OPT_BINARY = 0, OPT_ECHO = 1, OPT_SGA = 3, OPT_COM_PORT = 44;
constexpr uint8_t SET_BAUDRATE = 1, SET_DATASIZE = 2, SET_PARITY = 3, SET_STOPSIZE = 4, SET_CONTROL = 5,
                  NOTIFY_LINESTATE = 6, NOTIFY_MODEMSTATE = 7, FLOWCONTROL_SUSPEND = 8, FLOWCONTROL_RESUME = 9,
                  SET_LINESTATE_MASK = 10, SET_MODEMSTATE_MASK = 11, PURGE_DATA = 12;
constexpr uint8_t SERVER_OFFSET = 100;

#ifdef _WIN32
struct WsaInit {
    WsaInit() { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); }
    ~WsaInit() { WSACleanup(); }
};
#endif

class TcpSerial : public SerialHost {
public:
    TcpSerial(std::string bind, int port, bool rfc2217) : bind_(std::move(bind)), port_(port), rfc2217_(rfc2217) {}
    ~TcpSerial() override { stop(); }

    bool start(std::string* error) override {
#ifdef _WIN32
        static WsaInit wsa;
#endif
        listen_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_ == kBadSock) { *error = "socket() failed"; return false; }
        int yes = 1;
        setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof yes);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(uint16_t(port_));
        if (inet_pton(AF_INET, bind_.c_str(), &addr.sin_addr) != 1) { *error = "bad bind address " + bind_; return false; }
        if (::bind(listen_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0) {
            *error = "cannot listen on " + bind_ + ":" + std::to_string(port_) + " (port in use?)";
            close_sock(listen_);
            listen_ = kBadSock;
            return false;
        }
        ::listen(listen_, 1);
        running_ = true;
        thread_ = std::thread([this] { loop(); });
        return true;
    }

    void stop() override {
        if (!running_) return;
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (listen_ != kBadSock) close_sock(listen_);
        listen_ = kBadSock;
    }

    void write(const uint8_t* data, size_t len) override {
        std::lock_guard<std::mutex> lock(send_mutex_);
        if (client_ == kBadSock) return;
        if (!rfc2217_ || !std::memchr(data, IAC, len)) {
            send_all(data, len);
            return;
        }
        std::vector<uint8_t> esc;
        esc.reserve(len + 16);
        for (size_t i = 0; i < len; ++i) {
            esc.push_back(data[i]);
            if (data[i] == IAC) esc.push_back(IAC);
        }
        send_all(esc.data(), esc.size());
    }

    std::string describe() const override {
        std::string host = bind_ == "0.0.0.0" ? "localhost" : bind_;
        return (rfc2217_ ? "rfc2217://" : "socket://") + host + ":" + std::to_string(port_);
    }

private:
    void send_all(const uint8_t* p, size_t n) {  // send_mutex_ held
        while (n) {
            auto r = ::send(client_, reinterpret_cast<const char*>(p), int(n), 0);
            if (r <= 0) return;
            p += r;
            n -= size_t(r);
        }
    }

    void send_raw(std::initializer_list<uint8_t> bytes) {
        std::vector<uint8_t> v(bytes);
        std::lock_guard<std::mutex> lock(send_mutex_);
        if (client_ != kBadSock) send_all(v.data(), v.size());
    }

    void send_sub(uint8_t cmd, const std::vector<uint8_t>& value) {
        std::vector<uint8_t> v = {IAC, SB, OPT_COM_PORT, cmd};
        for (uint8_t b : value) {
            v.push_back(b);
            if (b == IAC) v.push_back(IAC);
        }
        v.push_back(IAC);
        v.push_back(SE);
        std::lock_guard<std::mutex> lock(send_mutex_);
        if (client_ != kBadSock) send_all(v.data(), v.size());
    }

    void loop() {
        while (running_) {
            fd_set rd;
            FD_ZERO(&rd);
            FD_SET(listen_, &rd);
            timeval tv{0, 100000};
            if (::select(int(listen_) + 1, &rd, nullptr, nullptr, &tv) <= 0) continue;
            sock_t c = ::accept(listen_, nullptr, nullptr);
            if (c == kBadSock) continue;
            int yes = 1;
            setsockopt(c, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&yes), sizeof yes);
            {
                std::lock_guard<std::mutex> lock(send_mutex_);
                client_ = c;
            }
            LOGI("%s: client connected", describe().c_str());
            push({SerialEvent::Kind::Connected, {}, false, false});
            serve();
            {
                std::lock_guard<std::mutex> lock(send_mutex_);
                close_sock(client_);
                client_ = kBadSock;
            }
            LOGI("%s: client disconnected", describe().c_str());
            push({SerialEvent::Kind::Disconnected, {}, false, false});
        }
    }

    void serve() {
        mode_ = Mode::Normal;
        sub_.clear();
        in_sub_ = false;
        dtr_ = rts_ = false;
        for (int i = 0; i < 256; ++i) we_[i] = they_[i] = false;
        if (rfc2217_) {
            // Offer binary mode and the COM port option.
            we_[OPT_BINARY] = we_[OPT_SGA] = we_[OPT_COM_PORT] = they_[OPT_BINARY] = true;
            send_raw({IAC, WILL, OPT_BINARY, IAC, DO, OPT_BINARY, IAC, WILL, OPT_SGA, IAC, WILL, OPT_COM_PORT});
        }
        uint8_t buf[4096];
        while (running_) {
            fd_set rd;
            FD_ZERO(&rd);
            FD_SET(client_, &rd);
            timeval tv{0, 100000};
            int r = ::select(int(client_) + 1, &rd, nullptr, nullptr, &tv);
            if (r < 0) return;
            if (r == 0) continue;
            auto n = ::recv(client_, reinterpret_cast<char*>(buf), sizeof buf, 0);
            if (n <= 0) return;
            if (!rfc2217_) {
                push({SerialEvent::Kind::Data, std::vector<uint8_t>(buf, buf + n), false, false});
            } else {
                telnet_input(buf, size_t(n));
            }
        }
    }

    void telnet_input(const uint8_t* p, size_t n) {
        std::vector<uint8_t> data;
        for (size_t i = 0; i < n; ++i) {
            uint8_t b = p[i];
            switch (mode_) {
            case Mode::Normal:
                if (b == IAC) mode_ = Mode::Iac;
                else if (in_sub_) sub_.push_back(b);
                else data.push_back(b);
                break;
            case Mode::Iac:
                mode_ = Mode::Normal;
                if (b == IAC) {
                    if (in_sub_) sub_.push_back(b);
                    else data.push_back(b);
                } else if (b == SB) {
                    in_sub_ = true;
                    sub_.clear();
                } else if (b == SE) {
                    in_sub_ = false;
                    if (!data.empty()) {  // keep ordering of data vs. control changes
                        push({SerialEvent::Kind::Data, data, false, false});
                        data.clear();
                    }
                    subnegotiation();
                } else if (b == DO || b == DONT || b == WILL || b == WONT) {
                    neg_cmd_ = b;
                    mode_ = Mode::Negotiate;
                }
                break;
            case Mode::Negotiate:
                mode_ = Mode::Normal;
                negotiate(neg_cmd_, b);
                break;
            }
        }
        if (!data.empty()) push({SerialEvent::Kind::Data, data, false, false});
    }

    void negotiate(uint8_t cmd, uint8_t opt) {
        bool supported = opt == OPT_BINARY || opt == OPT_SGA || opt == OPT_COM_PORT || (opt == OPT_ECHO && cmd == DO);
        if (cmd == WILL) {
            if (!they_[opt]) { they_[opt] = supported; send_raw({IAC, uint8_t(supported ? DO : DONT), opt}); }
        } else if (cmd == DO) {
            if (opt == OPT_ECHO) { send_raw({IAC, WONT, opt}); return; }  // we never echo
            if (!we_[opt]) { we_[opt] = supported; send_raw({IAC, uint8_t(supported ? WILL : WONT), opt}); }
            if (opt == OPT_COM_PORT) send_sub(NOTIFY_MODEMSTATE + SERVER_OFFSET, {0x30});  // DSR|CTS
        } else if (cmd == WONT) {
            they_[opt] = false;
        } else if (cmd == DONT) {
            we_[opt] = false;
        }
    }

    void subnegotiation() {
        if (sub_.size() < 2 || sub_[0] != OPT_COM_PORT) return;
        uint8_t cmd = sub_[1];
        std::vector<uint8_t> v(sub_.begin() + 2, sub_.end());
        switch (cmd) {
        case SET_BAUDRATE:
            if (v.size() >= 4) {
                uint32_t b = (uint32_t(v[0]) << 24) | (uint32_t(v[1]) << 16) | (uint32_t(v[2]) << 8) | v[3];
                if (b) baud_ = b;
            }
            send_sub(cmd + SERVER_OFFSET, {uint8_t(baud_ >> 24), uint8_t(baud_ >> 16), uint8_t(baud_ >> 8), uint8_t(baud_)});
            break;
        case SET_DATASIZE: send_sub(cmd + SERVER_OFFSET, {8}); break;
        case SET_PARITY: send_sub(cmd + SERVER_OFFSET, {1}); break;
        case SET_STOPSIZE: send_sub(cmd + SERVER_OFFSET, {1}); break;
        case SET_CONTROL: {
            uint8_t c = v.empty() ? 0 : v[0];
            bool lines_changed = false;
            switch (c) {
            case 8: dtr_ = true; lines_changed = true; break;
            case 9: dtr_ = false; lines_changed = true; break;
            case 11: rts_ = true; lines_changed = true; break;
            case 12: rts_ = false; lines_changed = true; break;
            case 0: c = 1; break;            // flow control query -> none
            case 7: c = dtr_ ? 8 : 9; break;  // DTR query
            case 10: c = rts_ ? 11 : 12; break;
            case 4: c = 6; break;            // break query -> off
            default: break;
            }
            if (lines_changed) push({SerialEvent::Kind::Lines, {}, dtr_, rts_});
            send_sub(SET_CONTROL + SERVER_OFFSET, {c});
            break;
        }
        case NOTIFY_LINESTATE: send_sub(cmd + SERVER_OFFSET, {0}); break;
        case NOTIFY_MODEMSTATE: send_sub(cmd + SERVER_OFFSET, {0x30}); break;
        case SET_LINESTATE_MASK:
        case SET_MODEMSTATE_MASK:
        case PURGE_DATA:
            send_sub(cmd + SERVER_OFFSET, v.empty() ? std::vector<uint8_t>{0} : std::vector<uint8_t>{v[0]});
            break;
        case FLOWCONTROL_SUSPEND:
        case FLOWCONTROL_RESUME:
            break;
        default: break;
        }
    }

    enum class Mode { Normal, Iac, Negotiate };

    std::string bind_;
    int port_;
    bool rfc2217_;
    sock_t listen_ = kBadSock, client_ = kBadSock;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex send_mutex_;

    Mode mode_ = Mode::Normal;
    uint8_t neg_cmd_ = 0;
    bool in_sub_ = false;
    std::vector<uint8_t> sub_;
    bool we_[256] = {}, they_[256] = {};
    bool dtr_ = false, rts_ = false;
    uint32_t baud_ = 115200;
};

}  // namespace

std::unique_ptr<SerialHost> make_tcp_serial(const std::string& bind_addr, int port, bool rfc2217) {
    return std::make_unique<TcpSerial>(bind_addr, port, rfc2217);
}

}  // namespace esp32sim
