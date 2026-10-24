#pragma once

#include <cstdarg>
#include <cstddef>

namespace sd_log {

// Append one CSV line under /usd/. Writes header once if the file is new/empty.
// Returns false if no SD card or open/write failed.
bool append_csv(const char* path, const char* header, const char* line);

// printf-style helper into a stack buffer then append_csv.
bool append_csvf(const char* path, const char* header, const char* fmt, ...);

bool card_present();

} // namespace sd_log
