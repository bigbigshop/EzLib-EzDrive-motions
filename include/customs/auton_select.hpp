#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

// Headless auton picker — no LVGL page (saves heap vs rd::Selector).
// Controller LEFT/RIGHT + Dash name label; autonomous() calls run().

namespace auton_select {

struct Routine {
    const char* name;
    std::function<void()> action;
};

void init(std::vector<Routine> routines);
void next(bool wrap = true);
void prev(bool wrap = true);
void run();
const char* name(); // never null; "No auton" if empty
std::size_t index();
std::size_t count();

} // namespace auton_select
