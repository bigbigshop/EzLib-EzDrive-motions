#pragma once

namespace telem_dash {

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
