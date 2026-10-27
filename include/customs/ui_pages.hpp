#pragma once

// Focus Dash or PID without destroying the other (malloc LVGL).
// Field-control teardown still releases both from telemetry_fn.

namespace ui_pages {

void show_dash();
void show_pid();

} // namespace ui_pages
