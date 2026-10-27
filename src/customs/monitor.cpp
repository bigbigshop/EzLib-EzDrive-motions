#include "main.h"
#include "customs/monitor.hpp"
#include "customs/auton_select.hpp"
#include "customs/boot_log.hpp"
#include "customs/telem_dash.hpp"
#include "customs/ui_pages.hpp"
#include "pid_tuner.hpp"

#include <cstdio>
#include <cstring>
#include <string>

// One Anim view + one lv_img; frames stay in flash. Do NOT create 10 rd::Image
// pages — that OOMs the 32KB LVGL heap.

bool autoChange = false;
int displayMenu = 0;
float globalTime = 0;
float elapsedSec = 0;

namespace {

constexpr int k_frame_count = 10;
constexpr uint32_t k_frame_ms = 20;

const lv_img_dsc_t* const k_frames[k_frame_count] = {
    &robot01, &robot02, &robot03, &robot04, &robot05,
    &robot06, &robot07, &robot08, &robot09, &robot10,
};

rd_view_t* anim_view = nullptr;
lv_obj_t* anim_img = nullptr;
int anim_frame = 0;
uint32_t last_frame_ms = 0;
bool anim_was_on = false;
bool views_btn_hidden = false;

// Hide robodash top-right view-switcher (32×32 @ TOP_RIGHT).
bool hide_robodash_views_btn() {
    if (views_btn_hidden) return true;
    lv_obj_t* scr = lv_scr_act();
    if (scr == nullptr) return false;
    const uint32_t n = lv_obj_get_child_cnt(scr);
    for (uint32_t i = 0; i < n; ++i) {
        lv_obj_t* ch = lv_obj_get_child(scr, i);
        if (ch == nullptr) continue;
        if (lv_obj_get_width(ch) != 32 || lv_obj_get_height(ch) != 32) continue;
        if (lv_obj_get_x(ch) < 430 || lv_obj_get_y(ch) > 12) continue;
        lv_obj_add_flag(ch, LV_OBJ_FLAG_HIDDEN);
        views_btn_hidden = true;
        return true;
    }
    return false;
}

bool ensure_anim_view() {
    if (anim_view != nullptr && anim_img != nullptr) return true;
    if (anim_view == nullptr) {
        anim_view = rd_view_create("Anim");
        if (anim_view == nullptr) return false;
        rd_view_set_anims(anim_view, RD_ANIM_OFF);
    }
    lv_obj_t* parent = rd_view_obj(anim_view);
    if (parent == nullptr) return false;
    if (anim_img == nullptr) {
        anim_img = lv_img_create(parent);
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
        anim_was_on = false;
        return;
    }
    if (!ensure_anim_view()) {
        autoChange = false;
        return;
    }

    if (!anim_was_on) {
        rd_view_focus(anim_view);
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
    if (anim_view == nullptr) return;
    rd_view_del(anim_view);
    anim_view = nullptr;
    anim_img = nullptr; // owned by the view
}

void uiTask_fn() {
    boot_log::log("uiTask:enter");
    // Controller lines start quickly; PID page waits so boot stays responsive.
    pros::delay(200);
    const uint32_t boot_ms = pros::millis();
    boot_log::log("uiTask:loop");

    uint32_t lastControllerTime = 0;
    static char pending[3][32] = {};
    static bool dirty[3] = {true, true, true}; // redraw after initialize() boot text
    static int next_line = 0;
    // Seed so first frame replaces "EzDrive boot / calibrating..." promptly.
    std::snprintf(pending[0], sizeof(pending[0]), "%-15.15s", "");
    std::snprintf(pending[1], sizeof(pending[1]), "%-15.15s", "");
    std::snprintf(pending[2], sizeof(pending[2]), "%-15.15s", "");

    while (true) {
        uint32_t now = pros::millis();
        // Hide robodash view menu once chrome exists.
        if (!views_btn_hidden && (now - boot_ms) >= 400 && (now - boot_ms) < 8000) {
            if (hide_robodash_views_btn()) {
                boot_log::log("uiTask:views_btn hidden");
            }
        }
        // Robot anim gated by autoChange inside.
        tick_robot_anim(now);

        // Controller text. Print a changed line only — never clear() on a timer.
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
                // Match / field-control: battery, auton, pose only — no tuner UI.
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
    // Dash already focused in initialize(); re-focus after settle in case
    // something else stole the view during calibrate.
    pros::delay(200);
    boot_log::log("telemetry:show_dash");
    ui_pages::show_dash();

    static bool was_field = false;
    while (true) {
        const bool field = pros::competition::is_connected();
        if (field) {
            if (!was_field) {
                was_field = true;
                boot_log::log("telemetry:field on — teardown heavy UI, keep Dash");
                destroy_robot_anim();
                pidTuner.release_ui();
                // No Selector page — keep Type R Dash (shows auton name).
                ui_pages::show_dash();
            }
            pros::delay(200);
            continue;
        }
        if (was_field) {
            was_field = false;
            boot_log::log("telemetry:field off — restore Dash");
            ui_pages::show_dash();
        }

        // Only refresh once boot finished (data panels exist).
        if (telem_dash::live()) {
            telem_dash::refresh();
        }
        pros::delay(150);
    }
}
