#include "main.h"
#include "customs/monitor.hpp"
#include "pid_tuner.hpp"

#include <cstdio>
#include <cstring>
#include <string>

// One Anim view + one lv_img; frames stay in flash. Do NOT create 10 rd::Image
// pages — that OOMs the 32KB LVGL heap.
rd::Console console;

bool autoChange = false;
int displayMenu = 0;
float globalTime = 0;
float elapsedSec = 0;

namespace {

constexpr int k_frame_count = 10;
constexpr uint32_t k_frame_ms = 100;

const lv_img_dsc_t* const k_frames[k_frame_count] = {
    &robot01, &robot02, &robot03, &robot04, &robot05,
    &robot06, &robot07, &robot08, &robot09, &robot10,
};

rd_view_t* anim_view = nullptr;
lv_obj_t* anim_img = nullptr;
int anim_frame = 0;
uint32_t last_frame_ms = 0;
bool anim_was_on = false;

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
    pros::delay(1000); // wait for LVGL / robodash to settle
    uint32_t lastControllerTime = 0;

    while (true) {
        uint32_t now = pros::millis();
        tick_robot_anim(now);

        // Controller text. Print a changed line only — never clear() on a timer.
        if (now - lastControllerTime > 60) {
            static char pending[3][32] = {};
            static bool dirty[3] = {};
            static int next_line = 0;

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
                std::string autonName = selector.get_auton()
                    .value_or(rd::Selector::routine_t{"No auton", [](){}}).name;
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
                std::string autonName = selector.get_auton()
                    .value_or(rd::Selector::routine_t{"No auton selected", [](){}}).name;
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

int get_safe_conf(pros::Distance& s) {
    int32_t conf = s.get_confidence();
    if (conf == PROS_ERR || conf < 0 || conf > 100) {
        return 0;
    }
    return conf;
}

void telemetry_fn() {
    float F_dist = 0.0f;
    float B_dist = 0.0f;
    float L_dist = 0.0f;
    float R_dist = 0.0f;

    int f_conf = 0, b_conf = 0, l_conf = 0, r_conf = 0;

    static bool was_field = false;
    while (true) {
        const bool field = pros::competition::is_connected();
        // Field control: stop hammering Console (temps / distance) — big LVGL churn.
        if (field) {
            if (!was_field) {
                was_field = true;
                destroy_robot_anim();
                pidTuner.release_ui(); // delete PID page — reclaim LVGL heap
                selector.focus();
            }
            pros::delay(200);
            continue;
        }
        if (was_field) {
            was_field = false;
            pidTuner.start(); // recreate PID page for practice
        }

        console.clear();
        console.printf("X: %5.2f,\t", chassis.getPose().x);
        console.printf("Y: %5.2f,\t", chassis.getPose().y);
        console.printf("Theta: %.2f\n", chassis.getPose().theta);
        console.println("");
        console.printf("LF: %.f,\t ", leftMotors.get_temperature(0));
        console.printf(" LM: %.f,\t", leftMotors.get_temperature(1));
        console.printf(" LB: %.f\n", leftMotors.get_temperature(2));

        console.printf("RF: %.f,\t ", rightMotors.get_temperature(0));
        console.printf("RM: %.f,\t", rightMotors.get_temperature(1));
        console.printf("RB: %.f\n", rightMotors.get_temperature(2));
        console.println("");

        console.printf("In_L: %.f,", left_intake.get_temperature());
        console.printf(" In_R: %.f,", right_intake.get_temperature());
        console.printf(" LRoll: %.f,", low_roller.get_temperature());
        console.printf(" HRoll: %.f\n", high_roller.get_temperature());
        console.println("");

        console.printf("Dist: %.2f\n", ezPID.get_traveled());

        lv_mem_monitor_t mon{};
        lv_mem_monitor(&mon);
        console.printf("LVGL %u%% used  free %u B\n", mon.used_pct, mon.free_size);
        console.println("");

        F_dist = get_kf_dist(dist_front);
        B_dist = get_kf_dist(dist_back);
        L_dist = get_kf_dist(dist_left);
        R_dist = get_kf_dist(dist_right);

        f_conf = get_safe_conf(dist_front);
        b_conf = get_safe_conf(dist_back);
        l_conf = get_safe_conf(dist_left);
        r_conf = get_safe_conf(dist_right);

        console.printf("F: %6.2f(%2d)\tB: %.2f(%2d)\n", F_dist, f_conf, B_dist, b_conf);
        console.printf("L: %6.2f(%2d)\tR: %.2f(%2d)\n", L_dist, l_conf, R_dist, r_conf);
        pros::delay(50);
    }
}
