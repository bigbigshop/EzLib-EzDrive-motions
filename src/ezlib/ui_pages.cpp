#include "ezlib/ui_pages.hpp"

#include "ezlib/boot_log.hpp"
#include "ezlib/telem_dash.hpp"
#include "main.h"
#include "ezlib/pid_tuner.hpp"

namespace ui_pages {

void show_dash() {
    boot_log::log("ui_pages:show_dash");
    // Keep PID page alive — only focus Dash.
    telem_dash::start();
    if (telem_dash::ready()) {
        telem_dash::focus();
    } else {
        boot_log::log("ui_pages:show_dash FAIL");
    }
}

void show_pid() {
    boot_log::log("ui_pages:show_pid");
    // Keep Dash alive — only focus PID.
    pidTuner.start();
    pidTuner.focus();
}

} // namespace ui_pages
