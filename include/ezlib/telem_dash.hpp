#pragma once

// Type R Dash: boot = centered gauge only; after calibrate → left batt gauge + data.

namespace telem_dash {

void start();          // centered ignition gauge (no data yet)
void finish_boot();    // after IMU calibrate — move gauge left (battery) + reveal data
void release();
void focus();
bool ready();
bool live();           // true once data panels are up
void refresh();

} // namespace telem_dash
