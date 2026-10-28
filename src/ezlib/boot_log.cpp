#include "ezlib/boot_log.hpp"

#include "ezlib/sd_log.hpp"
#include "pros/rtos.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace boot_log {
namespace {

constexpr const char* k_path = "/usd/boot_debug.log";
constexpr const char* k_header = "ms,msg\n";

// Raw write — do NOT require usd_is_installed() (often false/too early at boot).
bool write_line(const char* line) {
    if (line == nullptr) return false;
    std::FILE* f = std::fopen(k_path, "a");
    if (f == nullptr) return false;
    // Header only when file empty.
    if (std::ftell(f) == 0) {
        std::fputs(k_header, f);
    }
    std::fputs(line, f);
    if (line[0] != '\0' && line[std::strlen(line) - 1] != '\n') std::fputc('\n', f);
    std::fflush(f);
    std::fclose(f);
    return true;
}

void emit(const char* line) {
#if !BOOT_LOG_ENABLE
    (void)line;
    return;
#else
    std::printf("%s\n", line);
    std::fflush(stdout);
    write_line(line);
#endif
}

} // namespace

void ensure_sd(const char* tag) {
#if !BOOT_LOG_ENABLE
    (void)tag;
    return;
#else
    char line[192];
    for (int i = 0; i < 40; ++i) { // up to ~2s
        std::snprintf(line, sizeof(line), "%lu,sd_probe[%s] try=%d usd=%d",
                      static_cast<unsigned long>(pros::millis()),
                      tag != nullptr ? tag : "?", i,
                      sd_log::card_present() ? 1 : 0);
        if (write_line(line)) {
            std::printf("%s\n", line);
            std::fflush(stdout);
            log("sd_probe OK after try=%d", i);
            return;
        }
        pros::delay(50);
    }
    log("sd_probe FAIL after 40 tries usd=%d", sd_log::card_present() ? 1 : 0);
#endif
}

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

} // namespace boot_log

#if BOOT_LOG_ENABLE
// Runs before most other static constructors (priority 101). If this line
// appears on SD but initialize:enter does not, crash is in a later global ctor
// (historically rd::Selector / rd::Console + large LVGL heap).
__attribute__((constructor(101))) static void boot_log_very_early() {
    std::FILE* f = std::fopen("/usd/boot_debug.log", "a");
    if (f == nullptr) return;
    if (std::ftell(f) == 0) std::fputs("ms,msg\n", f);
    std::fputs("0,CTOR_early_before_robodash\n", f);
    std::fflush(f);
    std::fclose(f);
}
#endif
