#include "pid_tuner.hpp"
#include "customs/monitor.hpp"

#include <cmath>
#include <cstdio>
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
    ACT_FIELD_NEXT
};

BumpEvent g_bumps[2];
ActionEvent g_actions[12];
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

void PidTuner::print_now() {
    const std::string snippet = drive.constants_snippet();
    std::printf("%s", snippet.c_str());
    std::fflush(stdout);
    console.focus();
    console.clear();
    console.println("=== EzDrive constants (paste into initialize) ===");
    console.println(snippet);
    std::snprintf(note_text_, sizeof(note_text_), "printed — see Console / serial");
    note_dirty_ = true;
}

void PidTuner::test_drive(double inches) {
    if (drive.is_busy()) drive.cancel_movement();
    drive.drive(inches, 90, 1800, true);
}

void PidTuner::test_turn(double degrees) {
    if (drive.is_busy()) drive.cancel_movement();
    drive.turn(degrees, 90, 1500, true);
}

void PidTuner::zero_pose() {
    drive.set_pose(0, 0, 0);
    drive.reset_pid();
    std::snprintf(note_text_, sizeof(note_text_), "pose set to 0,0,0 (odom only)");
    note_dirty_ = true;
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
        case ACT_FIELD_PREV: self->step_field(-1); break;
        case ACT_FIELD_NEXT: self->step_field(+1); break;
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
        std::snprintf(buf, sizeof(buf), "err drive %+.2f in  turn %+.1f deg%s", drive.drive_pid().error,
                      drive.turn_pid().error, controller_mode_ ? "  CTRL" : "");
        lv_label_set_text(live_, buf);
    }
    if (note_ != nullptr && note_dirty_) {
        lv_label_set_text(note_, note_text_);
        note_dirty_ = false;
    }
}

void PidTuner::start() {
    if (started_) return;
    started_ = true;
    g_action_count = 0;

    // Compact UI only — two-column grid OOMs the 32KB LVGL heap (pc:3807204 /
    // lv_obj_class_create_obj). Long field names still show on the active row.
    view_ = rd_view_create("PID");
    if (view_ == nullptr) return;
    rd_view_set_anims(view_, RD_ANIM_OFF);
    lv_obj_t* parent = rd_view_obj(view_);
    if (parent == nullptr) return;

    lv_obj_set_style_bg_color(parent, lv_color_hex(0x121418), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    lv_obj_t* title = lv_label_create(parent);
    if (title != nullptr) {
        lv_label_set_text(title, "EzDrive PID  < > field   +/- value");
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
    add_action("+90", ACT_TURN_POS, 168, 152, 56);
    add_action("-90", ACT_TURN_NEG, 232, 152, 56);
    add_action("Print", ACT_PRINT, 296, 152, 64);

    // Single action row — Pose0 stays above robodash chrome and remains tappable.
    add_action("Ctrl", ACT_CONTROLLER, 8, 188, 72);
    add_action("Pose0", ACT_POSE0, 88, 188, 72);

    note_ = lv_label_create(parent);
    if (note_ != nullptr) {
        lv_label_set_text(note_, note_text_);
        lv_obj_set_pos(note_, 8, 220);
        style_label(note_);
    }

    lv_timer_create(on_timer, 100, this);
    refresh();
}

void PidTuner::toggle_controller_mode() {
    controller_mode_ = !controller_mode_;
    if (controller_mode_) drive.cancel_movement();
    std::snprintf(note_text_, sizeof(note_text_), controller_mode_ ? "controller tuner on" : "controller tuner off");
    note_dirty_ = true;
}

bool PidTuner::suppress_arcade() const { return controller_mode_ || drive.is_busy(); }

void PidTuner::format_controller(char line0[32], char line1[32], char line2[32]) const {
    std::snprintf(line0, 32, "%s %.3g", field_name(field_), field_value(field_));
    std::snprintf(line1, 32, "e %+.2f %+.1f", drive.drive_pid().error, drive.turn_pid().error);
    std::snprintf(line2, 32, "睇Github");
}

void PidTuner::handle_controller(pros::Controller& controller) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) &&
        !controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        toggle_controller_mode();
        controller.rumble(".");
        return;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        field_ = (field_ + F_COUNT - 1) % F_COUNT;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { field_ = (field_ + 1) % F_COUNT; }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) bump(field_, +1);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) bump(field_, -1);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) test_drive(12);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) test_turn(90);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) print_now();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) test_drive(-12);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) test_turn(-90);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) zero_pose();
}

} // namespace ez_compat
