#include "ezlib/ui_pages.hpp"

#include "ezlib/boot_log.hpp"
#include "ezlib/telem_dash.hpp"
#include "main.h"
#include "ezlib/pid_tuner.hpp"

namespace ui_pages {

void show_dash() {
    boot_log::log("ui_pages:show_dash");
    pidTuner.hide(); // 內含 set_gauge_hidden(false)
    telem_dash::start();
    if (telem_dash::ready()) {
        telem_dash::set_gauge_hidden(false);
        telem_dash::focus();
    } else {
        boot_log::log("ui_pages:show_dash FAIL");
    }
}

void show_pid() {
    boot_log::log("ui_pages:show_pid");
    pidTuner.start();
    if (!pidTuner.alive()) {
        boot_log::log("ui_pages:show_pid FAIL");
        return;
    }
    pidTuner.focus(); // 內含 set_gauge_hidden(true)
}

} // namespace ui_pages
