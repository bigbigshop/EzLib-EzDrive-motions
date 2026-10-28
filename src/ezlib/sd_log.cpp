#include "ezlib/sd_log.hpp"

#include "pros/misc.h"

#include <cstdio>
#include <cstring>

namespace sd_log {

bool card_present() { return pros::c::usd_is_installed(); }

namespace {

bool file_needs_header(const char* path) {
    std::FILE* f = std::fopen(path, "r");
    if (f == nullptr) return true;
    char probe[8] = {};
    const std::size_t n = std::fread(probe, 1, sizeof(probe), f);
    std::fclose(f);
    return n == 0;
}

} // namespace

bool append_csv(const char* path, const char* header, const char* line) {
    if (!card_present() || path == nullptr || line == nullptr) return false;

    const bool write_header = file_needs_header(path);
    std::FILE* f = std::fopen(path, "a");
    if (f == nullptr) return false;

    if (write_header && header != nullptr) {
        std::fputs(header, f);
        if (header[0] != '\0' && header[std::strlen(header) - 1] != '\n') std::fputc('\n', f);
    }
    std::fputs(line, f);
    if (line[0] != '\0' && line[std::strlen(line) - 1] != '\n') std::fputc('\n', f);
    std::fflush(f);
    std::fclose(f);
    return true;
}

bool append_csvf(const char* path, const char* header, const char* fmt, ...) {
    char line[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    return append_csv(path, header, line);
}

bool CsvWriter::begin(const char* path, const char* header) {
    end();
    if (!card_present() || path == nullptr) return false;
    const bool write_header = file_needs_header(path);
    f_ = std::fopen(path, "a");
    if (f_ == nullptr) return false;
    if (write_header && header != nullptr) {
        std::fputs(header, f_);
        if (header[0] != '\0' && header[std::strlen(header) - 1] != '\n') std::fputc('\n', f_);
    }
    return true;
}

bool CsvWriter::linef(const char* fmt, ...) {
    if (f_ == nullptr || fmt == nullptr) return false;
    char line[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    std::fputs(line, f_);
    if (line[0] != '\0' && line[std::strlen(line) - 1] != '\n') std::fputc('\n', f_);
    return true;
}

void CsvWriter::end() {
    if (f_ == nullptr) return;
    std::fflush(f_);
    std::fclose(f_);
    f_ = nullptr;
}

} // namespace sd_log
