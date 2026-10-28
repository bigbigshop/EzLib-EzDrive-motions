#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdio>

namespace sd_log {

// Append one CSV line under /usd/. Writes header once if the file is new/empty.
// Returns false if no SD card or open/write failed.
bool append_csv(const char* path, const char* header, const char* line);

// printf-style helper into a stack buffer then append_csv.
bool append_csvf(const char* path, const char* header, const char* fmt, ...);

bool card_present();

// Keep one file open for high-rate sampling (e.g. 10–20 ms PID ticks).
// Prefer this over append_csvf in a tight loop (avoids open/close every sample).
class CsvWriter {
    public:
        bool begin(const char* path, const char* header);
        bool linef(const char* fmt, ...);
        void end();
        bool ok() const { return f_ != nullptr; }

    private:
        std::FILE* f_ = nullptr;
};

} // namespace sd_log
