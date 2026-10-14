#pragma once
#include <cstdarg>

namespace esp32sim {

enum class LogLevel { Error = 0, Warn = 1, Info = 2, Debug = 3, Trace = 4 };

void set_log_level(LogLevel level);
LogLevel log_level();
void log_msg(LogLevel level, const char* fmt, ...)
#if defined(__MINGW32__)
    __attribute__((format(gnu_printf, 2, 3)))
#elif defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

#define LOGE(...) ::esp32sim::log_msg(::esp32sim::LogLevel::Error, __VA_ARGS__)
#define LOGW(...) ::esp32sim::log_msg(::esp32sim::LogLevel::Warn, __VA_ARGS__)
#define LOGI(...) ::esp32sim::log_msg(::esp32sim::LogLevel::Info, __VA_ARGS__)
#define LOGD(...) \
    do { if (::esp32sim::log_level() >= ::esp32sim::LogLevel::Debug) ::esp32sim::log_msg(::esp32sim::LogLevel::Debug, __VA_ARGS__); } while (0)
#define LOGT(...) \
    do { if (::esp32sim::log_level() >= ::esp32sim::LogLevel::Trace) ::esp32sim::log_msg(::esp32sim::LogLevel::Trace, __VA_ARGS__); } while (0)

}  // namespace esp32sim
