#include "customs/ui_pages.hpp"

#include "customs/boot_log.hpp"
#include "customs/telem_dash.hpp"
#include "main.h"
#include "pid_tuner.hpp"

namespace ui_pages {

void show_dash() {
    boot_log::log("ui_pages:show_dash");
    pidTuner.release_ui();
    telem_dash::start();
    telem_dash::focus();
    boot_log::lvgl("after_show_dash");
}

void show_pid() {
    boot_log::log("ui_pages:show_pid");
    telem_dash::release();
    pidTuner.start();
    pidTuner.focus();
    boot_log::lvgl("after_show_pid");
}

} // namespace ui_pages
