#include "main.h"
#include "ezlib/monitor.hpp"
#include "ezlib/auton_select.hpp"
#include "ezlib/boot_log.hpp"
#include "ezlib/telem_dash.hpp"
#include "ezlib/ui_pages.hpp"
#include "ezlib/pid_tuner.hpp"

#include "liblvgl/lvgl.h"

#include <cstdio>
#include <cstring>
#include <string>

// 動畫：全螢幕 overlay + 一個 lv_img；幀資料存於 flash（src/assets/robotXX.c）

bool autoChange = false;
int displayMenu = 0;
float globalTime = 0;
float elapsedSec = 0;

namespace {

// ----- 變更動畫幀數／速度請修改此處 -----
constexpr int k_frame_count = 10;      // 須與下方 k_frames 數量一致
constexpr uint32_t k_frame_ms = 10;    // 每幀間隔（ms）；數值愈小愈快

const lv_img_dsc_t* const k_frames[k_frame_count] = {
    &robot01, &robot02, &robot03, &robot04, &robot05,
    &robot06, &robot07, &robot08, &robot09, &robot10,
};


lv_obj_t* anim_page = nullptr;
lv_obj_t* anim_img = nullptr;
int anim_frame = 0;
uint32_t last_frame_ms = 0;
bool anim_was_on = false;

bool ensure_anim_view() {
    if (anim_page != nullptr && anim_img != nullptr) return true;
    lv_obj_t* scr = lv_scr_act();
    if (scr == nullptr) return false;

    if (anim_page == nullptr) {
        anim_page = lv_obj_create(scr);
        if (anim_page == nullptr) return false;
        lv_obj_set_size(anim_page, 480, 240);
        lv_obj_set_pos(anim_page, 0, 0);
        lv_obj_clear_flag(anim_page, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(anim_page, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(anim_page, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(anim_page, 0, 0);
        lv_obj_set_style_pad_all(anim_page, 0, 0);
        lv_obj_add_flag(anim_page, LV_OBJ_FLAG_HIDDEN);
    }
    if (anim_img == nullptr) {
        anim_img = lv_img_create(anim_page);
        if (anim_img == nullptr) return false;
        lv_img_set_src(anim_img, k_frames[0]);
        // Asset is 480x270; brain is 480x240 — pin top-left, clip bottom.
        lv_obj_set_pos(anim_img, 0, 0);
    }
    return true;
}

void tick_robot_anim(uint32_t now) {
    const bool want =
        autoChange && !pidTuner.controller_mode() && !pros::competition::is_connected();

    if (!want) {
        if (anim_was_on) {
            anim_was_on = false;
            if (anim_page != nullptr) lv_obj_add_flag(anim_page, LV_OBJ_FLAG_HIDDEN);
            ui_pages::show_dash();
        }
        return;
    }
    if (!ensure_anim_view()) {
        autoChange = false;
        return;
    }

    if (!anim_was_on) {
        pidTuner.hide();
        lv_obj_clear_flag(anim_page, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(anim_page);
        anim_was_on = true;
        last_frame_ms = now;
    }

    if (now - last_frame_ms < k_frame_ms) return;
    last_frame_ms = now;
    anim_frame = (anim_frame + 1) % k_frame_count;
    lv_img_set_src(anim_img, k_frames[anim_frame]);
}

} // namespace

void destroy_robot_anim() {
    autoChange = false;
    anim_was_on = false;
    if (anim_page == nullptr) return;
    lv_obj_del(anim_page);
    anim_page = nullptr;
    anim_img = nullptr;
}

void uiTask_fn() {
    boot_log::log("uiTask:enter");
    pros::delay(200);
    boot_log::log("uiTask:loop");

    uint32_t lastControllerTime = 0;
    static char pending[3][32] = {};
    static bool dirty[3] = {true, true, true};
    static int next_line = 0;
    std::snprintf(pending[0], sizeof(pending[0]), "%-15.15s", "");
    std::snprintf(pending[1], sizeof(pending[1]), "%-15.15s", "");
    std::snprintf(pending[2], sizeof(pending[2]), "%-15.15s", "");

    while (true) {
        uint32_t now = pros::millis();
        tick_robot_anim(now);

        if (now - lastControllerTime > 60) {
            auto stage = [&](int line, const char* text) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%-15.15s", text != nullptr ? text : "");
                if (std::strncmp(pending[line], buf, sizeof(buf)) != 0) {
                    std::snprintf(pending[line], sizeof(pending[line]), "%s", buf);
                    dirty[line] = true;
                }
            };

            char raw0[32];
            char raw1[32];
            char raw2[32];
            const bool field = pros::competition::is_connected();

            if (field) {
                std::string autonName = auton_select::name();
                std::snprintf(raw0, sizeof(raw0), "B:%3.0f C:%3.0f",
                              (double)controller.get_battery_capacity(),
                              (double)pros::battery::get_capacity());
                std::snprintf(raw1, sizeof(raw1), "%s", autonName.c_str());
                const auto pose = chassis.getPose();
                std::snprintf(raw2, sizeof(raw2), "X%.0f Y%.0f T%.0f", pose.x, pose.y, pose.theta);
            } else if (pidTuner.controller_mode()) {
                pidTuner.format_controller(raw0, raw1, raw2);
            } else if (displayMenu == 0) {
                elapsedSec = (now - globalTime) / 1000;
                std::string autonName = auton_select::name();
                std::snprintf(raw0, sizeof(raw0), "T:%4.0f", elapsedSec);
                std::snprintf(raw1, sizeof(raw1), "B:%3.0f C:%3.0f",
                              (double)controller.get_battery_capacity(),
                              (double)pros::battery::get_capacity());
                std::snprintf(raw2, sizeof(raw2), "%s", autonName.c_str());
            } else if (displayMenu == 1) {
                std::snprintf(raw0, sizeof(raw0), "D:%7.2f", ezPID.get_traveled());
                std::snprintf(raw1, sizeof(raw1), "T:%7.2f", chassis.getPose().theta);
                std::snprintf(raw2, sizeof(raw2), " ");
            } else {
                std::snprintf(raw0, sizeof(raw0), "X:%7.2f", chassis.getPose().x);
                std::snprintf(raw1, sizeof(raw1), "Y:%7.2f", chassis.getPose().y);
                std::snprintf(raw2, sizeof(raw2), "T:%7.2f", chassis.getPose().theta);
            }
            stage(0, raw0);
            stage(1, raw1);
            stage(2, raw2);

            for (int attempt = 0; attempt < 3; ++attempt) {
                const int line = (next_line + attempt) % 3;
                if (!dirty[line]) continue;
                controller.print(line, 0, "%s", pending[line]);
                dirty[line] = false;
                next_line = (line + 1) % 3;
                break;
            }
            lastControllerTime = now;
        }

        pros::delay(20);
    }
}

void telemetry_fn() {
    boot_log::log("telemetry:enter");
    for (int attempt = 0; attempt < 10 && !telem_dash::ready(); ++attempt) {
        pros::delay(300);
        boot_log::log("telemetry:finish_boot try=%d", attempt);
        telem_dash::finish_boot();
        if (!telem_dash::ready()) {
            controller.print(2, 0, "%-15.15s", telem_dash::last_error());
        }
    }
    if (telem_dash::ready()) {
        ui_pages::show_dash();
        boot_log::log("telemetry:dash live");
    } else {
        // 僅於失敗時在手掣顯示原因；成功則交由 uiTask 顯示正常三行
        controller.print(2, 0, "%-15.15s", telem_dash::last_error());
        boot_log::log("telemetry:dash FAILED %s", telem_dash::last_error());
    }

    static bool was_field = false;
    while (true) {
        const bool field = pros::competition::is_connected();
        if (field) {
            if (!was_field) {
                was_field = true;
                boot_log::log("telemetry:field on — teardown heavy UI, keep Dash");
                destroy_robot_anim();
                pidTuner.release_ui();
                if (telem_dash::ready()) ui_pages::show_dash();
            }
            pros::delay(200);
            continue;
        }
        if (was_field) {
            was_field = false;
            boot_log::log("telemetry:field off — restore Dash");
            if (telem_dash::ready()) ui_pages::show_dash();
        }

        if (telem_dash::live()) {
            telem_dash::refresh();
        }
        pros::delay(150);
    }
}
