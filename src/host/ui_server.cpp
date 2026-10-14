// Built-in web UI server. Plain HTTP/1.1 with Connection: close, one thread per
// connection; /events stays open as a Server-Sent Events stream.
#include "host/ui_server.hpp"

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>

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
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
using sock_t = int;
static constexpr sock_t kBadSock = -1;
static void close_sock(sock_t s) { ::close(s); }
#endif
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace esp32sim {

extern const UiAsset kUiAssets[];
extern const size_t kUiAssetCount;

const UiAsset* find_ui_asset(const std::string& path) {
    for (size_t i = 0; i < kUiAssetCount; ++i)
        if (path == kUiAssets[i].path) return &kUiAssets[i];
    return nullptr;
}

namespace {

constexpr size_t kSerialHistory = 64 * 1024;

#ifdef _WIN32
struct WsaInit {
    WsaInit() { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); }
    ~WsaInit() { WSACleanup(); }
};
#endif

bool send_all(sock_t s, const char* p, size_t n) {
    while (n) {
        auto r = ::send(s, p, int(n), MSG_NOSIGNAL);
        if (r <= 0) return false;
        p += r;
        n -= size_t(r);
    }
    return true;
}
bool send_str(sock_t s, const std::string& str) { return send_all(s, str.data(), str.size()); }

void respond(sock_t s, int code, const char* status, const char* type, const std::string& body) {
    std::string h = "HTTP/1.1 " + std::to_string(code) + " " + status + "\r\nContent-Type: " + type +
                    "\r\nContent-Length: " + std::to_string(body.size()) +
                    "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
    send_str(s, h);
    send_str(s, body);
}

const char* content_type(const std::string& path) {
    auto ends = [&](const char* ext) {
        size_t n = std::strlen(ext);
        return path.size() >= n && path.compare(path.size() - n, n, ext) == 0;
    };
    if (ends(".html")) return "text/html; charset=utf-8";
    if (ends(".js")) return "text/javascript; charset=utf-8";
    if (ends(".css")) return "text/css; charset=utf-8";
    if (ends(".json")) return "application/json";
    if (ends(".png")) return "image/png";
    if (ends(".jpg")) return "image/jpeg";
    if (ends(".svg")) return "image/svg+xml";
    return "application/octet-stream";
}

std::string base64(const char* p, size_t n) {
    static const char* t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((n + 2) / 3 * 4);
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = uint32_t(uint8_t(p[i])) << 16;
        if (i + 1 < n) v |= uint32_t(uint8_t(p[i + 1])) << 8;
        if (i + 2 < n) v |= uint8_t(p[i + 2]);
        out += t[(v >> 18) & 63];
        out += t[(v >> 12) & 63];
        out += i + 1 < n ? t[(v >> 6) & 63] : '=';
        out += i + 2 < n ? t[v & 63] : '=';
    }
    return out;
}

}  // namespace

UiServer::UiServer(std::string bind, int port) : bind_(std::move(bind)), port_(port) {}
UiServer::~UiServer() { stop(); }

bool UiServer::start(std::string* error) {
#ifdef _WIN32
    static WsaInit wsa;
#endif
    sock_t ls = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ls == kBadSock) { *error = "socket() failed"; return false; }
#ifndef _WIN32
    int yes = 1;  // on Windows SO_REUSEADDR would let two servers share the port
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof yes);
#endif
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(uint16_t(port_));
    if (inet_pton(AF_INET, bind_.c_str(), &addr.sin_addr) != 1) {
        *error = "bad bind address " + bind_;
        close_sock(ls);
        return false;
    }
    if (::bind(ls, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 || ::listen(ls, 16) != 0) {
        *error = "cannot listen on " + bind_ + ":" + std::to_string(port_) + " (port in use?)";
        close_sock(ls);
        return false;
    }
    listen_ = intptr_t(ls);
    running_ = true;
    thread_ = std::thread([this] { accept_loop(); });
    return true;
}

void UiServer::stop() {
    if (!running_) return;
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    close_sock(sock_t(listen_));
    // Connection threads notice running_ within their select/wait timeouts.
    for (int i = 0; i < 50 && connections_ > 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

std::string UiServer::url() const {
    std::string host = bind_ == "0.0.0.0" ? "localhost" : bind_;
    return "http://" + host + ":" + std::to_string(port_) + "/";
}

void UiServer::set_catalog(std::string json) {
    std::lock_guard<std::mutex> lock(m_);
    catalog_ = std::move(json);
}

void UiServer::publish_state(std::string json) {
    {
        std::lock_guard<std::mutex> lock(m_);
        if (json == state_) return;
        state_ = std::move(json);
        ++state_version_;
    }
    cv_.notify_all();
}

void UiServer::publish_serial(const uint8_t* data, size_t len) {
    {
        std::lock_guard<std::mutex> lock(m_);
        serial_.append(reinterpret_cast<const char*>(data), len);
        if (serial_.size() > 2 * kSerialHistory) {
            size_t drop = serial_.size() - kSerialHistory;
            serial_.erase(0, drop);
            serial_base_ += drop;
        }
    }
    cv_.notify_all();
}

bool UiServer::poll(UiCommand& cmd) {
    std::lock_guard<std::mutex> lock(m_);
    if (commands_.empty()) return false;
    cmd = std::move(commands_.front());
    commands_.pop_front();
    return true;
}

void UiServer::accept_loop() {
    sock_t ls = sock_t(listen_);
    while (running_) {
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(ls, &rd);
        timeval tv{0, 100000};
        if (::select(int(ls) + 1, &rd, nullptr, nullptr, &tv) <= 0) continue;
        sock_t c = ::accept(ls, nullptr, nullptr);
        if (c == kBadSock) continue;
        ++connections_;
        std::thread([this, c] {
            handle(intptr_t(c));
            close_sock(c);
            --connections_;
        }).detach();
    }
}

void UiServer::handle(intptr_t sock) {
    sock_t s = sock_t(sock);
#ifdef _WIN32
    DWORD tmo = 5000;
#else
    timeval tmo{5, 0};
#endif
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tmo), sizeof tmo);

    // Read headers, then the body by Content-Length.
    std::string buf;
    char tmp[4096];
    size_t hdr_end;
    while ((hdr_end = buf.find("\r\n\r\n")) == std::string::npos) {
        auto n = ::recv(s, tmp, sizeof tmp, 0);
        if (n <= 0 || buf.size() > 65536) return;
        buf.append(tmp, size_t(n));
    }
    Request req;
    size_t sp1 = buf.find(' '), sp2 = buf.find(' ', sp1 + 1);
    if (sp1 == std::string::npos || sp2 == std::string::npos || sp2 > hdr_end) return;
    req.method = buf.substr(0, sp1);
    req.path = buf.substr(sp1 + 1, sp2 - sp1 - 1);
    size_t content_length = 0;
    std::string headers = buf.substr(0, hdr_end);
    for (auto& ch : headers) ch = char(std::tolower(static_cast<unsigned char>(ch)));
    size_t cl = headers.find("\r\ncontent-length:");
    if (cl != std::string::npos) content_length = size_t(std::strtoul(headers.c_str() + cl + 17, nullptr, 10));
    if (content_length > 1 << 20) return respond(s, 413, "Payload Too Large", "text/plain", "too large\n");
    req.body = buf.substr(hdr_end + 4);
    while (req.body.size() < content_length) {
        auto n = ::recv(s, tmp, sizeof tmp, 0);
        if (n <= 0) return;
        req.body.append(tmp, size_t(n));
    }
    size_t q = req.path.find('?');
    if (q != std::string::npos) req.path.resize(q);

    if (req.method == "GET") {
        if (req.path == "/events") return serve_events(sock);
        if (req.path == "/api/catalog") {
            std::string body;
            {
                std::lock_guard<std::mutex> lock(m_);
                body = catalog_;
            }
            return respond(s, 200, "OK", "application/json", body);
        }
        std::string path = req.path == "/" ? "index.html" : req.path.substr(1);
        if (const UiAsset* a = find_ui_asset(path))
            return respond(s, 200, "OK", content_type(path),
                           std::string(reinterpret_cast<const char*>(a->data), a->size));
        return respond(s, 404, "Not Found", "text/plain", "not found\n");
    }
    if (req.method == "POST" && req.path.rfind("/api/", 0) == 0) {
        UiCommand cmd;
        cmd.name = req.path.substr(5);
        std::string err;
        if (!req.body.empty() && !Json::parse(req.body, cmd.args, &err))
            return respond(s, 400, "Bad Request", "text/plain", "bad JSON: " + err + "\n");
        {
            std::lock_guard<std::mutex> lock(m_);
            commands_.push_back(std::move(cmd));
        }
        return respond(s, 200, "OK", "application/json", "{}");
    }
    respond(s, 405, "Method Not Allowed", "text/plain", "method not allowed\n");
}

void UiServer::serve_events(intptr_t sock) {
    sock_t s = sock_t(sock);
    if (!send_str(s, "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\n"
                     "Connection: keep-alive\r\n\r\nretry: 1000\n\n"))
        return;
    uint64_t seen_state = ~uint64_t(0);
    uint64_t serial_pos;
    {
        std::lock_guard<std::mutex> lock(m_);
        serial_pos = serial_base_ + (serial_.size() > kSerialHistory ? serial_.size() - kSerialHistory : 0);
    }
    auto last_send = std::chrono::steady_clock::now();
    while (running_) {
        std::string out;
        {
            std::unique_lock<std::mutex> lock(m_);
            cv_.wait_for(lock, std::chrono::milliseconds(500), [&] {
                return !running_ || state_version_ != seen_state || serial_base_ + serial_.size() != serial_pos;
            });
            if (state_version_ != seen_state) {
                out += "event: state\ndata: " + state_ + "\n\n";
                seen_state = state_version_;
            }
            uint64_t end = serial_base_ + serial_.size();
            if (serial_pos < serial_base_) serial_pos = serial_base_;
            if (end != serial_pos) {
                size_t off = size_t(serial_pos - serial_base_);
                out += "event: serial\ndata: " + base64(serial_.data() + off, serial_.size() - off) + "\n\n";
                serial_pos = end;
            }
        }
        auto now = std::chrono::steady_clock::now();
        if (out.empty() && now - last_send > std::chrono::seconds(15)) out = ": keepalive\n\n";
        if (out.empty()) continue;
        if (!send_str(s, out)) return;
        last_send = now;
    }
}

}  // namespace esp32sim
