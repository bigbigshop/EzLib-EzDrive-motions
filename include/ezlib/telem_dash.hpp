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
// PID overlay 蓋住時隱藏電量錶／指針（避免 transform 畫穿）
void set_gauge_hidden(bool hidden);
// Short ASCII reason for last start() failure ("" if ok / never tried).
const char* last_error();

} // namespace telem_dash
