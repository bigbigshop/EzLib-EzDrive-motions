#pragma once

namespace telem_dash {

// Loading page theme — set from main.cpp before finish_boot / show_dash.
// 1 = none (skip boot animation)
// 2 = Type R (gauges ignition)
// 3 = Spear of Longinus
enum class LoadingTheme : int {
    None = 1,
    TypeR = 2,
    Spear = 3,
};

void set_loading_theme(LoadingTheme theme);
LoadingTheme loading_theme();

void start();
void finish_boot();
void release();
void focus();
bool ready();
bool live();
void refresh();
void set_gauge_hidden(bool hidden);
const char* last_error();

} // namespace telem_dash
