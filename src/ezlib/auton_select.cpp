#include "ezlib/auton_select.hpp"

#include "ezlib/boot_log.hpp"

#include <cstdio>

namespace auton_select {
namespace {

std::vector<Routine> routines_;
std::size_t index_ = 0;

void persist() {
    if (routines_.empty()) return;
    FILE* f = std::fopen("/usd/auton_sel.txt", "w");
    if (f == nullptr) return;
    std::fprintf(f, "%zu\n", static_cast<unsigned long>(index_));
    std::fclose(f);
}

void load() {
    FILE* f = std::fopen("/usd/auton_sel.txt", "r");
    if (f == nullptr) return;
    unsigned long v = 0;
    if (std::fscanf(f, "%lu", &v) == 1 && !routines_.empty()) {
        index_ = static_cast<std::size_t>(v) % routines_.size();
    }
    std::fclose(f);
}

} // namespace

void init(std::vector<Routine> routines) {
    routines_ = std::move(routines);
    index_ = 0;
    load();
    boot_log::log("auton_select:init n=%zu idx=%zu %s", routines_.size(), index_, name());
}

void next(bool wrap) {
    if (routines_.empty()) return;
    if (index_ + 1 < routines_.size()) {
        ++index_;
    } else if (wrap) {
        index_ = 0;
    }
    persist();
    boot_log::log("auton_select:next %s", name());
}

void prev(bool wrap) {
    if (routines_.empty()) return;
    if (index_ > 0) {
        --index_;
    } else if (wrap) {
        index_ = routines_.size() - 1;
    }
    persist();
    boot_log::log("auton_select:prev %s", name());
}

void run() {
    if (routines_.empty() || routines_[index_].action == nullptr) {
        boot_log::log("auton_select:run EMPTY");
        return;
    }
    boot_log::log("auton_select:run %s", name());
    routines_[index_].action();
}

const char* name() {
    if (routines_.empty() || routines_[index_].name == nullptr) return "No auton";
    return routines_[index_].name;
}

std::size_t index() { return index_; }
std::size_t count() { return routines_.size(); }

} // namespace auton_select
