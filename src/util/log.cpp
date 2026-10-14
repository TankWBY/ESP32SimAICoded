#include "util/log.hpp"

#include <chrono>
#include <cstdio>
#include <mutex>

namespace esp32sim {

namespace {
LogLevel g_level = LogLevel::Info;
std::mutex g_mutex;
const auto g_start = std::chrono::steady_clock::now();
}  // namespace

void set_log_level(LogLevel level) { g_level = level; }
LogLevel log_level() { return g_level; }

void log_msg(LogLevel level, const char* fmt, ...) {
    if (level > g_level) return;
    static const char* names[] = {"E", "W", "I", "D", "T"};
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - g_start).count();
    std::lock_guard<std::mutex> lock(g_mutex);
    std::fprintf(stderr, "[%9.3f %s] ", t, names[int(level)]);
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
    std::fflush(stderr);
}

}  // namespace esp32sim
