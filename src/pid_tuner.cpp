#include "pid_tuner.hpp"
#include "customs/monitor.hpp"
#include "customs/objDeclare.hpp"
#include "customs/sd_log.hpp"
#include "customs/telem_dash.hpp"
#include "customs/ui_pages.hpp"

#include "pros/rtos.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <string>

namespace ez_compat {

namespace {

struct BumpEvent {
        PidTuner* self;
        int direction;
};

struct ActionEvent {
        PidTuner* self;
        int action;
};

enum Action {
    ACT_DRIVE_POS = 1,
    ACT_DRIVE_NEG,
    ACT_TURN_POS,
    ACT_TURN_NEG,
    ACT_PRINT,
    ACT_CONTROLLER,
    ACT_POSE0,
    ACT_FIELD_PREV,
    ACT_FIELD_NEXT,
    ACT_OFFSET_CCW,
    ACT_OFFSET_CW,
    ACT_DASH
};

constexpr const char* k_pid_csv = "/usd/pid_tune_log.csv";
constexpr const char* k_pid_header =
    "timestamp_ms,wall_time,event,cmd_inches,cmd_heading,"
    "drive_kp,drive_ki,drive_kd,drive_error,drive_sm,drive_bg,drive_vl,"
    "turn_kp,turn_ki,turn_kd,turn_error,turn_sm,turn_bg,turn_vl,"
    "head_kp,head_ki,head_kd,head_error,head_sm,head_bg,head_vl,"
    "odom_x,odom_y,odom_theta,odom_theta_delta,"
    "imu_heading,imu_rotation,imu_rotation_delta,traveled";

constexpr const char* k_offset_csv = "/usd/findingOffset.csv";
constexpr const char* k_offset_header =
    "timestamp_ms,wall_time,dir,rev,quarter,odom_x,odom_y,odom_theta,imu_heading,imu_rotation";

void wall_time_str(char* out, std::size_t n) {
    const std::time_t t = std::time(nullptr);
    if (t > 0) {
        std::tm* tm = std::localtime(&t);
        if (tm != nullptr) {
            std::snprintf(out, n, "%04d-%02d-%02d_%02d:%02d:%02d", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                          tm->tm_hour, tm->tm_min, tm->tm_sec);
            return;
        }
    }
    // V5 often has no RTC — fall back to boot millis.
    std::snprintf(out, n, "boot_%lu", static_cast<unsigned long>(pros::millis()));
}

BumpEvent g_bumps[2];
ActionEvent g_actions[14];
int g_action_count = 0;

lv_obj_t* make_button(lv_obj_t* parent, const char* text, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                      lv_event_cb_t cb, void* user) {
    lv_obj_t* button = lv_btn_create(parent);
    if (button == nullptr) return nullptr;
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, w, h);
    lv_obj_t* label = lv_label_create(button);
    if (label != nullptr) {
        lv_label_set_text(label, text);
        lv_obj_center(label);
    }
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, user);
    return button;
}

void style_label(lv_obj_t* label) {
    if (label == nullptr) return;
    lv_obj_set_style_text_color(label, lv_color_hex(0xF4F7FB), 0);
}

} // namespace

const char* PidTuner::field_name(int field) const {
    // Long names for brain + controller (small / big / vel, not sm/bg).
    switch (field) {
        case F_DRIVE_P: return "DRIVE kP";
        case F_DRIVE_I: return "DRIVE kI";
        case F_DRIVE_D: return "DRIVE kD";
        case F_DRIVE_SMALL: return "DRIVE small";
        case F_DRIVE_BIG: return "DRIVE big";
        case F_DRIVE_VEL: return "DRIVE vel";
        case F_TURN_P: return "TURN kP";
        case F_TURN_I: return "TURN kI";
        case F_TURN_D: return "TURN kD";
        case F_TURN_SMALL: return "TURN small";
        case F_TURN_BIG: return "TURN big";
        case F_TURN_VEL: return "TURN vel";
        case F_HEAD_P: return "HEAD kP";
        case F_HEAD_I: return "HEAD kI";
        case F_HEAD_D: return "HEAD kD";
        default: return "?";
    }
}

double PidTuner::field_step(int field) const {
    switch (field) {
        case F_DRIVE_P:
        case F_TURN_P:
        case F_HEAD_P: return 0.05;
        case F_DRIVE_I:
        case F_TURN_I:
        case F_HEAD_I: return 0.01;
        case F_DRIVE_D:
        case F_TURN_D:
        case F_HEAD_D: return 0.5;
        case F_DRIVE_SMALL:
        case F_DRIVE_BIG: return 0.1;
        case F_TURN_SMALL:
        case F_TURN_BIG: return 0.5;
        case F_DRIVE_VEL: return 0.01;
        case F_TURN_VEL: return 0.05;
        default: return 0.1;
    }
}

double PidTuner::field_value(int field) const {
    const PID& drive_pid = drive.drive_pid();
    const PID& turn_pid = drive.turn_pid();
    const PID& heading_pid = drive.heading_pid();
    switch (field) {
        case F_DRIVE_P: return drive_pid.kp;
        case F_DRIVE_I: return drive_pid.ki;
        case F_DRIVE_D: return drive_pid.kd;
        case F_DRIVE_SMALL: return drive_pid.small_error;
        case F_DRIVE_BIG: return drive_pid.big_error;
        case F_DRIVE_VEL: return drive_pid.velocity_threshold;
        case F_TURN_P: return turn_pid.kp;
        case F_TURN_I: return turn_pid.ki;
        case F_TURN_D: return turn_pid.kd;
        case F_TURN_SMALL: return turn_pid.small_error;
        case F_TURN_BIG: return turn_pid.big_error;
        case F_TURN_VEL: return turn_pid.velocity_threshold;
        case F_HEAD_P: return heading_pid.kp;
        case F_HEAD_I: return heading_pid.ki;
        case F_HEAD_D: return heading_pid.kd;
        default: return 0;
    }
}

void PidTuner::set_field_value(int field, double value) {
    if (value < 0) value = 0;
    PID& drive_pid = drive.drive_pid();
    PID& turn_pid = drive.turn_pid();
    PID& heading_pid = drive.heading_pid();
    switch (field) {
        case F_DRIVE_P: drive_pid.kp = value; break;
        case F_DRIVE_I: drive_pid.ki = value; break;
        case F_DRIVE_D: drive_pid.kd = value; break;
        case F_DRIVE_SMALL: drive_pid.small_error = value; break;
        case F_DRIVE_BIG: drive_pid.big_error = value; break;
        case F_DRIVE_VEL: drive_pid.set_velocity_threshold(value); break;
        case F_TURN_P: turn_pid.kp = value; break;
        case F_TURN_I: turn_pid.ki = value; break;
        case F_TURN_D: turn_pid.kd = value; break;
        case F_TURN_SMALL: turn_pid.small_error = value; break;
        case F_TURN_BIG: turn_pid.big_error = value; break;
        case F_TURN_VEL: turn_pid.set_velocity_threshold(value); break;
        case F_HEAD_P: heading_pid.kp = value; break;
        case F_HEAD_I: heading_pid.ki = value; break;
        case F_HEAD_D: heading_pid.kd = value; break;
        default: break;
    }
}

void PidTuner::bump(int field, int direction) {
    if (field < 0 || field >= F_COUNT) return;
    set_field_value(field, field_value(field) + direction * field_step(field));
}

void PidTuner::set_note(const char* text) {
    std::snprintf(note_text_, sizeof(note_text_), "%s", text != nullptr ? text : "");
    note_dirty_ = true;
}

void PidTuner::print_now() {
    const std::string snippet = drive.constants_snippet();
    // Console page removed (LVGL heap). Dump to USB terminal; snapshot still → SD CSV.
    std::printf("=== YTT-EzLib constants (paste into initialize) ===\n%s\n", snippet.c_str());
    std::fflush(stdout);
    log_pid_event("print", 0, 0, chassis.getPose().theta, imu.get_rotation(), imu.get_heading());
    set_note(sd_log::card_present() ? "printed USB+SD" : "printed USB (no SD)");
}

void PidTuner::log_pid_event(const char* event, double cmd_inches, double cmd_heading, double start_odom_theta,
                             double start_imu_rot, double start_imu_heading) {
    (void)start_imu_heading;
    const auto& d = drive.drive_pid();
    const auto& t = drive.turn_pid();
    const auto& h = drive.heading_pid();
    const lemlib::Pose pose = chassis.getPose();
    const double imu_rot = imu.get_rotation();
    const double imu_head = imu.get_heading();
    const double odom_delta = pose.theta - start_odom_theta;
    const double imu_delta = imu_rot - start_imu_rot;

    char wall[32];
    wall_time_str(wall, sizeof(wall));

    const bool ok = sd_log::append_csvf(
        k_pid_csv, k_pid_header,
        "%lu,%s,%s,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f",
        static_cast<unsigned long>(pros::millis()), wall, event, cmd_inches, cmd_heading, d.kp, d.ki, d.kd, d.error,
        d.small_error, d.big_error, d.velocity_threshold, t.kp, t.ki, t.kd, t.error, t.small_error, t.big_error,
        t.velocity_threshold, h.kp, h.ki, h.kd, h.error, h.small_error, h.big_error, h.velocity_threshold, pose.x,
        pose.y, pose.theta, odom_delta, imu_head, imu_rot, imu_delta, drive.get_traveled());

    if (std::strcmp(event, "drive") == 0) {
        last_imu_delta_ = imu_delta;
        last_odom_delta_ = odom_delta;
        last_imu_heading_ = imu_head;
        last_odom_theta_ = pose.theta;
        has_drive_sample_ = true;
        char note[96];
        std::snprintf(note, sizeof(note), "dIMU%+.2f dOdom%+.2f%s", imu_delta, odom_delta, ok ? " SD" : " noSD");
        set_note(note);
    } else if (!ok) {
        set_note("SD write failed / no card");
    }
}

void PidTuner::log_pid_run(const char* event_prefix, double cmd_inches, double cmd_heading, double start_odom_theta,
                           double start_imu_rot, double start_imu_heading) {
    (void)start_imu_heading;
    constexpr int k_sample_ms = 15; // 10–20ms band — captures oscillation
    char wall[32];
    wall_time_str(wall, sizeof(wall));

    sd_log::CsvWriter writer;
    const bool sd_ok = writer.begin(k_pid_csv, k_pid_header);
    int samples = 0;

    while (drive.is_busy()) {
        const auto& d = drive.drive_pid();
        const auto& t = drive.turn_pid();
        const auto& h = drive.heading_pid();
        const lemlib::Pose pose = chassis.getPose();
        const double imu_rot = imu.get_rotation();
        const double imu_head = imu.get_heading();
        const double odom_delta = pose.theta - start_odom_theta;
        const double imu_delta = imu_rot - start_imu_rot;

        if (sd_ok) {
            char event[24];
            std::snprintf(event, sizeof(event), "%s_tick", event_prefix);
            writer.linef("%lu,%s,%s,%.3f,%.3f,"
                         "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                         "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                         "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                         "%.4f,%.4f,%.4f,%.4f,"
                         "%.4f,%.4f,%.4f,%.4f",
                         static_cast<unsigned long>(pros::millis()), wall, event, cmd_inches, cmd_heading, d.kp, d.ki,
                         d.kd, d.error, d.small_error, d.big_error, d.velocity_threshold, t.kp, t.ki, t.kd, t.error,
                         t.small_error, t.big_error, t.velocity_threshold, h.kp, h.ki, h.kd, h.error, h.small_error,
                         h.big_error, h.velocity_threshold, pose.x, pose.y, pose.theta, odom_delta, imu_head, imu_rot,
                         imu_delta, drive.get_traveled());
            ++samples;
        }
        pros::delay(k_sample_ms);
    }

    writer.end();
    // Final settle row (same as before) for quick end-of-move lookup.
    log_pid_event(event_prefix, cmd_inches, cmd_heading, start_odom_theta, start_imu_rot, start_imu_heading);

    if (sd_ok) {
        char note[96];
        std::snprintf(note, sizeof(note), "%s SD %d ticks + end", event_prefix, samples);
        set_note(note);
    } else if (!sd_log::card_present()) {
        set_note("done (no SD)");
    }
}

void PidTuner::test_drive(double inches) {
    if (offset_running_) return;
    if (drive.is_busy()) drive.cancel_movement();
    const double start_odom = chassis.getPose().theta;
    const double start_imu_rot = imu.get_rotation();
    const double start_imu_head = imu.get_heading();
    drive.drive(inches, 90, 1800, true);
    pros::Task([this, inches, start_odom, start_imu_rot, start_imu_head]() {
        log_pid_run("drive", inches, 0, start_odom, start_imu_rot, start_imu_head);
    });
}

void PidTuner::test_turn(double degrees) {
    if (offset_running_) return;
    if (drive.is_busy()) drive.cancel_movement();
    const double start_odom = chassis.getPose().theta;
    const double start_imu_rot = imu.get_rotation();
    const double start_imu_head = imu.get_heading();
    drive.turn(degrees, 90, 1500, true);
    pros::Task([this, degrees, start_odom, start_imu_rot, start_imu_head]() {
        log_pid_run("turn", 0, degrees, start_odom, start_imu_rot, start_imu_head);
    });
}

void PidTuner::zero_pose() {
    drive.set_pose(0, 0, 0);
    drive.reset_pid();
    set_note("pose set to 0,0,0 (odom only)");
}

void PidTuner::run_offset_spins(int direction) {
    if (direction != 1 && direction != -1) direction = -1;
    if (offset_running_) {
        set_note("offset test already running");
        return;
    }
    if (!sd_log::card_present()) {
        set_note("need SD for findingOffset.csv");
    }
    offset_running_ = true;
    if (drive.is_busy()) drive.cancel_movement();

    pros::Task([this, direction]() {
        // 10 revs = 40 × (±90°) so wrap PID can complete full circles.
        const char* dir_name = direction > 0 ? "CCW" : "CW";
        const double step_deg = direction > 0 ? 90.0 : -90.0;

        drive.set_pose(0, 0, 0);
        drive.reset_pid();
        pros::delay(50);

        auto log_rev = [dir_name](int rev, int quarter) {
            const lemlib::Pose p = chassis.getPose();
            char wall[32];
            wall_time_str(wall, sizeof(wall));
            sd_log::append_csvf(k_offset_csv, k_offset_header, "%lu,%s,%s,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f",
                                static_cast<unsigned long>(pros::millis()), wall, dir_name, rev, quarter, p.x, p.y,
                                p.theta, imu.get_heading(), imu.get_rotation());
        };

        log_rev(0, 0);
        char start_note[64];
        std::snprintf(start_note, sizeof(start_note), "offset: %s x10...", dir_name);
        set_note(start_note);

        for (int rev = 1; rev <= 10; ++rev) {
            for (int q = 1; q <= 4; ++q) {
                const double th = chassis.getPose().theta;
                drive.turn(th + step_deg, 80, 4000, false);
                drive.waitUntilDone();
                log_rev(rev, q);
            }
            char note[64];
            std::snprintf(note, sizeof(note), "offset %s %d/10", dir_name, rev);
            set_note(note);
        }

        offset_running_ = false;
        set_note(sd_log::card_present() ? "findingOffset.csv done" : "spins done (no SD)");
    });
}

void PidTuner::step_field(int direction) {
    field_ = (field_ + direction + F_COUNT) % F_COUNT;
}

void PidTuner::on_bump_event(lv_event_t* event) {
    auto* bump = static_cast<BumpEvent*>(lv_event_get_user_data(event));
    if (bump != nullptr && bump->self != nullptr) bump->self->bump(bump->self->field_, bump->direction);
}

void PidTuner::on_action_event(lv_event_t* event) {
    auto* action = static_cast<ActionEvent*>(lv_event_get_user_data(event));
    if (action == nullptr || action->self == nullptr) return;
    PidTuner* self = action->self;
    switch (action->action) {
        case ACT_DRIVE_POS: self->test_drive(12); break;
        case ACT_DRIVE_NEG: self->test_drive(-12); break;
        case ACT_TURN_POS: self->test_turn(90); break;
        case ACT_TURN_NEG: self->test_turn(-90); break;
        case ACT_PRINT: self->print_now(); break;
        case ACT_CONTROLLER: self->toggle_controller_mode(); break;
        case ACT_POSE0: self->zero_pose(); break;
        case ACT_OFFSET_CCW: self->run_offset_spins(+1); break;
        case ACT_OFFSET_CW: self->run_offset_spins(-1); break;
        case ACT_DASH:
            self->controller_mode_ = false;
            self->drive.cancel_movement();
            std::snprintf(self->note_text_, sizeof(self->note_text_), "back to Dash");
            self->note_dirty_ = true;
            ui_pages::show_dash();
            break;
        case ACT_FIELD_PREV:
            self->step_field(-1);
            self->zero_pose();
            break;
        case ACT_FIELD_NEXT:
            self->step_field(+1);
            self->zero_pose();
            break;
        default: break;
    }
}

void PidTuner::on_timer(lv_timer_t* timer) {
    auto* self = static_cast<PidTuner*>(timer->user_data);
    if (self != nullptr) self->refresh();
}

void PidTuner::refresh() {
    char buf[64];
    if (field_label_ != nullptr) {
        std::snprintf(buf, sizeof(buf), "%s", field_name(field_));
        lv_label_set_text(field_label_, buf);
    }
    if (value_label_ != nullptr) {
        std::snprintf(buf, sizeof(buf), "%.4f   step %.3g", field_value(field_), field_step(field_));
        lv_label_set_text(value_label_, buf);
    }
    if (live_ != nullptr) {
        const lemlib::Pose pose = chassis.getPose();
        if (has_drive_sample_) {
            std::snprintf(buf, sizeof(buf), "imu %+.1f odom %+.1f dI%+.2f dO%+.2f", last_imu_heading_,
                          last_odom_theta_, last_imu_delta_, last_odom_delta_);
        } else {
            std::snprintf(buf, sizeof(buf), "imu %+.1f odom %+.1f eD%+.2f eT%+.1f", imu.get_heading(), pose.theta,
                          drive.drive_pid().error, drive.turn_pid().error);
        }
        lv_label_set_text(live_, buf);
    }
    if (note_ != nullptr && note_dirty_) {
        lv_label_set_text(note_, note_text_);
        note_dirty_ = false;
    }
}

void PidTuner::start() {
    if (started_) return;
    // Mutual exclusion with Dash — only one heavy page in LVGL heap.
    telem_dash::release();
    started_ = true;
    g_action_count = 0;

    // Compact UI only — two-column grid OOMs the 32KB LVGL heap (pc:3807204 /
    // lv_obj_class_create_obj). Long field names still show on the active row.
    view_ = rd_view_create("PID");
    if (view_ == nullptr) {
        started_ = false;
        return;
    }
    rd_view_set_anims(view_, RD_ANIM_OFF);
    lv_obj_t* parent = rd_view_obj(view_);
    if (parent == nullptr) return;

    lv_obj_set_style_bg_color(parent, lv_color_hex(0x121418), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    lv_obj_t* title = lv_label_create(parent);
    if (title != nullptr) {
        lv_label_set_text(title, "YTT-EzLib PID  < > field   +/- value");
        lv_obj_set_pos(title, 8, 4);
        style_label(title);
    }

    live_ = lv_label_create(parent);
    if (live_ != nullptr) {
        lv_label_set_text(live_, "err drive 0.00 in  turn 0.0 deg");
        lv_obj_set_pos(live_, 8, 28);
        style_label(live_);
    }

    field_label_ = lv_label_create(parent);
    if (field_label_ != nullptr) {
        lv_label_set_text(field_label_, "DRIVE kP");
        lv_obj_set_pos(field_label_, 8, 56);
        style_label(field_label_);
    }

    value_label_ = lv_label_create(parent);
    if (value_label_ != nullptr) {
        lv_label_set_text(value_label_, "0");
        lv_obj_set_pos(value_label_, 8, 80);
        style_label(value_label_);
    }

    auto add_action = [&](const char* text, int action, lv_coord_t x, lv_coord_t y, lv_coord_t w) {
        if (g_action_count >= static_cast<int>(sizeof(g_actions) / sizeof(g_actions[0]))) return;
        g_actions[g_action_count] = {this, action};
        make_button(parent, text, x, y, w, 28, on_action_event, &g_actions[g_action_count]);
        ++g_action_count;
    };

    add_action("<", ACT_FIELD_PREV, 8, 112, 44);
    add_action(">", ACT_FIELD_NEXT, 60, 112, 44);
    g_bumps[0] = {this, -1};
    make_button(parent, "-", 120, 112, 44, 28, on_bump_event, &g_bumps[0]);
    g_bumps[1] = {this, +1};
    make_button(parent, "+", 172, 112, 44, 28, on_bump_event, &g_bumps[1]);

    add_action("+12in", ACT_DRIVE_POS, 8, 152, 72);
    add_action("-12in", ACT_DRIVE_NEG, 88, 152, 72);
    add_action("h+90", ACT_TURN_POS, 168, 152, 56);
    add_action("h-90", ACT_TURN_NEG, 232, 152, 56);
    add_action("Print", ACT_PRINT, 296, 152, 64);

    // Single action row — Dash returns to practice dashboard (releases PID page).
    add_action("Ctrl", ACT_CONTROLLER, 8, 188, 56);
    add_action("Pose0", ACT_POSE0, 72, 188, 56);
    add_action("L10", ACT_OFFSET_CCW, 136, 188, 48);
    add_action("R10", ACT_OFFSET_CW, 192, 188, 48);
    add_action("Dash", ACT_DASH, 248, 188, 56);

    note_ = lv_label_create(parent);
    if (note_ != nullptr) {
        lv_label_set_text(note_, note_text_);
        lv_obj_set_pos(note_, 8, 220);
        style_label(note_);
    }

    timer_ = lv_timer_create(on_timer, 100, this);
    refresh();
}

void PidTuner::focus() {
    if (view_ != nullptr) rd_view_focus(view_);
}

void PidTuner::toggle_controller_mode() {
    controller_mode_ = !controller_mode_;
    if (controller_mode_) {
        drive.cancel_movement();
        ui_pages::show_pid(); // drop Dash, ensure PID view
        std::snprintf(note_text_, sizeof(note_text_), "controller tuner on");
        note_dirty_ = true;
    } else {
        std::snprintf(note_text_, sizeof(note_text_), "controller tuner off");
        note_dirty_ = true;
        ui_pages::show_dash(); // release PID page
    }
}

void PidTuner::force_off() {
    if (offset_running_) {
        drive.cancel_movement();
        offset_running_ = false;
    }
    if (controller_mode_) {
        controller_mode_ = false;
        drive.cancel_movement();
        set_note("tuner off (field control)");
    }
}

void PidTuner::release_ui() {
    force_off();
    if (timer_ != nullptr) {
        lv_timer_del(timer_);
        timer_ = nullptr;
    }
    if (view_ != nullptr) {
        rd_view_del(view_);
        view_ = nullptr;
    }
    field_label_ = nullptr;
    value_label_ = nullptr;
    live_ = nullptr;
    note_ = nullptr;
    started_ = false;
    g_action_count = 0;
}

bool PidTuner::suppress_arcade() const { return controller_mode_ || drive.is_busy() || offset_running_; }

void PidTuner::format_controller(char line0[32], char line1[32], char line2[32]) const {
    std::snprintf(line0, 32, "%s %.3g", field_name(field_), field_value(field_));
    if (has_drive_sample_) {
        std::snprintf(line1, 32, "dI%+.1f dO%+.1f", last_imu_delta_, last_odom_delta_);
    } else {
        std::snprintf(line1, 32, "imu%+.1f o%+.1f", imu.get_heading(), chassis.getPose().theta);
    }
    std::snprintf(line2, 32, "L2/R2 10spin");
}

void PidTuner::handle_controller(pros::Controller& controller) {
    // Exit on X alone. Enter is X+A in opcontrol — never bind drive to A.
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            toggle_controller_mode();
            controller.rumble(".");
        }
        return;
    }
    if (offset_running_) return;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        field_ = (field_ + F_COUNT - 1) % F_COUNT;
        zero_pose();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        field_ = (field_ + 1) % F_COUNT;
        zero_pose();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) bump(field_, +1);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) bump(field_, -1);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) zero_pose();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) test_drive(12);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) test_drive(-12);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) test_turn(-90);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) test_turn(90);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) run_offset_spins(+1); // left / CCW
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) run_offset_spins(-1); // right / CW
}

} // namespace ez_compat
