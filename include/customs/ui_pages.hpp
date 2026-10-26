#pragma once

// PID ↔ Dash mutual exclusion (only one heavy view in LVGL heap).
// Practice default = Dash. X+A / Dash "PID" → PID. DOWN → Dash.

namespace ui_pages {

void show_dash();
void show_pid();

} // namespace ui_pages
