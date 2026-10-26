#pragma once

// Practice-field Brain dashboard. Mutually exclusive with PID (see ui_pages).
// Fixed labels + batt bar — no Console clear churn.
// Motor/DIST layout: include/customs/dash_hardware_config.hpp (edit tables only).

namespace telem_dash {

void start();
void release();
void focus();
bool ready();
void refresh();

} // namespace telem_dash
