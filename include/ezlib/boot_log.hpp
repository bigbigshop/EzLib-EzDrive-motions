#pragma once

#include <cstdint>

// Lightweight breadcrumbs for boot / freeze debugging.
// - SD only: append /usd/boot_debug.log (NO printf — USB COBS ser_write data-aborts
//   were seen in User Initialization at cobs_encode_measure / PC ~0x0390c572).
// Toggle with BOOT_LOG_ENABLE (1=on).

#ifndef BOOT_LOG_ENABLE
#define BOOT_LOG_ENABLE 1
#endif

namespace boot_log {

void log(const char* fmt, ...);
// Retry fopen(/usd/boot_debug.log) for ~2s — call BEFORE creating Dash UI.
void ensure_sd(const char* tag);

} // namespace boot_log
