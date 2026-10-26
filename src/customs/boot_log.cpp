#include "customs/boot_log.hpp"

#include "customs/sd_log.hpp"
#include "liblvgl/lvgl.h"
#include "pros/rtos.hpp"

#include <cstdarg>
#include <cstdio>

namespace boot_log {
namespace {

constexpr const char* k_path = "/usd/boot_debug.log";
constexpr const char* k_header = "ms,msg";

void emit(const char* line) {
#if !BOOT_LOG_ENABLE
    (void)line;
    return;
#else
    std::printf("%s\n", line);
    std::fflush(stdout);
    if (sd_log::card_present()) {
        sd_log::append_csv(k_path, k_header, line);
    }
#endif
}

} // namespace

void log(const char* fmt, ...) {
#if !BOOT_LOG_ENABLE
    (void)fmt;
    return;
#else
    char msg[160];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    char line[192];
    std::snprintf(line, sizeof(line), "%lu,%s",
                  static_cast<unsigned long>(pros::millis()), msg);
    emit(line);
#endif
}

void lvgl(const char* tag) {
#if !BOOT_LOG_ENABLE
    (void)tag;
    return;
#else
    lv_mem_monitor_t mon{};
    lv_mem_monitor(&mon);
    const uint32_t used = mon.total_size - mon.free_size;
    log("lvgl[%s] used=%lu free=%lu total=%lu frag=%u%%",
        tag != nullptr ? tag : "?",
        static_cast<unsigned long>(used),
        static_cast<unsigned long>(mon.free_size),
        static_cast<unsigned long>(mon.total_size),
        static_cast<unsigned>(mon.frag_pct));
#endif
}

void heartbeat(const char* where) {
#if !BOOT_LOG_ENABLE
    (void)where;
    return;
#else
    static uint32_t last_ms = 0;
    const uint32_t now = pros::millis();
    if (now - last_ms < 2000) return;
    last_ms = now;
    lvgl(where != nullptr ? where : "hb");
#endif
}

} // namespace boot_log
