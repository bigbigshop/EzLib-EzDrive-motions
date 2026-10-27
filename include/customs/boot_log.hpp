#pragma once

#include <cstdint>

// Lightweight breadcrumbs for boot / freeze debugging.
// - Always: printf → PROS terminal (USB)
// - If SD present: append /usd/boot_debug.log
// Toggle with BOOT_LOG_ENABLE (1=on).

#ifndef BOOT_LOG_ENABLE
#define BOOT_LOG_ENABLE 1
#endif

namespace boot_log {

void log(const char* fmt, ...);
void lvgl(const char* tag); // LVGL heap: used/free/total/frag
void heartbeat(const char* where); // millis + LVGL snapshot
// Retry fopen(/usd/boot_debug.log) for ~2s — call BEFORE creating robodash views.
void ensure_sd(const char* tag);

} // namespace boot_log
