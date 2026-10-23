#include "customs/pid_tuner.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ez_compat {

namespace {

struct BumpEvent {
        PidTuner* self;
        int field;
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
    ACT_POSE0
};

BumpEvent g_bumps[24];
int g_bump_count = 0;
ActionEvent g_actions[8];
int g_action_count = 0;

lv_obj_t* make_button(lv_obj_t* parent, const char* text, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                      lv_event_cb_t cb, void* user) {
    lv_obj_t* button = lv_btn_create(parent);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, w, h);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, user);
    return button;
}

void style_label(lv_obj_t* label) {
    lv_obj_set_style_text_color(label, lv_color_hex(0xF4F7FB), 0);
}

} // namespace

const char* PidTuner::field_name(int field) const {
    switch (field) {
        case F_DRIVE_P: return "D kP";
        case F_DRIVE_I: return "D kI";
        case F_DRIVE_D: return "D kD";
        case F_DRIVE_SMALL: return "D sm";
        case F_DRIVE_BIG: return "D bg";
        case F_DRIVE_VEL: return "D vl";
        case F_TURN_P: return "T kP";
        case F_TURN_I: return "T kI";
        case F_TURN_D: return "T kD";
        case F_TURN_SMALL: return "T sm";
        case F_TURN_BIG: return "T bg";
        case F_TURN_VEL: return "T vl";
        case F_HEAD_P: return "H kP";
        case F_HEAD_I: return "H kI";
        case F_HEAD_D: return "H kD";
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
    const double next = field_value(field) + direction * field_step(field);
    set_field_value(field, next);
}

void PidTuner::print_now() {
    drive.print_constants();
    std::snprintf(note_text_, sizeof(note_text_), "printed to terminal — paste into initialize()");
    note_dirty_ = true;
}

void PidTuner::test_drive(double inches) {
    if (drive.is_busy()) return;
    drive.drive(inches, 90, 1800, true);
}

void PidTuner::test_turn(double degrees) {
    if (drive.is_busy()) return;
    drive.turn(degrees, 90, 1500, true);
}

void PidTuner::zero_pose() {
    drive.set_pose(0, 0, 0);
    std::snprintf(note_text_, sizeof(note_text_), "pose set to 0,0,0");
    note_dirty_ = true;
}

void PidTuner::on_bump_event(lv_event_t* event) {
    auto* bump = static_cast<BumpEvent*>(lv_event_get_user_data(event));
    if (bump != nullptr && bump->self != nullptr) bump->self->bump(bump->field, bump->direction);
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
        default: break;
    }
}

void PidTuner::on_timer(lv_timer_t* timer) {
    auto* self = static_cast<PidTuner*>(timer->user_data);
    if (self != nullptr) self->refresh();
}

void PidTuner::refresh() {
    char buf[48];
    for (int field = F_DRIVE_P; field <= F_TURN_VEL; ++field) {
        if (value_labels_[field] == nullptr) continue;
        std::snprintf(buf, sizeof(buf), "%.3f", field_value(field));
        lv_label_set_text(value_labels_[field], buf);
    }
    if (live_ != nullptr) {
        std::snprintf(buf, sizeof(buf), "err  drive %+.2f in   turn %+.1f deg%s", drive.drive_pid().error,
                      drive.turn_pid().error, controller_mode_ ? "   CTRL" : "");
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
    view_ = rd_view_create("PID");
    if (view_ == nullptr) return;
    rd_view_set_anims(view_, RD_ANIM_OFF);
    lv_obj_t* parent = rd_view_obj(view_);
    if (parent == nullptr) return;

    lv_obj_set_style_bg_color(parent, lv_color_hex(0x121418), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    lv_obj_t* title = lv_label_create(parent);
    lv_label_set_text(title, "EzDrive PID   inches / deg   no SD");
    lv_obj_set_pos(title, 8, 2);
    style_label(title);

    live_ = lv_label_create(parent);
    lv_label_set_text(live_, "err  drive 0.00 in   turn 0.0 deg");
    lv_obj_set_pos(live_, 8, 20);
    style_label(live_);

    const int drive_fields[] = {F_DRIVE_P, F_DRIVE_I, F_DRIVE_D, F_DRIVE_SMALL, F_DRIVE_BIG, F_DRIVE_VEL};
    const int turn_fields[] = {F_TURN_P, F_TURN_I, F_TURN_D, F_TURN_SMALL, F_TURN_BIG, F_TURN_VEL};
    const char* row_names[] = {"kP", "kI", "kD", "small", "big", "vel"};

    auto build_column = [&](int x, const char* header, const int* fields) {
        lv_obj_t* head = lv_label_create(parent);
        lv_label_set_text(head, header);
        lv_obj_set_pos(head, x, 40);
        style_label(head);
        for (int row = 0; row < 6; ++row) {
            const int y = 58 + row * 22;
            const int field = fields[row];
            lv_obj_t* name = lv_label_create(parent);
            lv_label_set_text(name, row_names[row]);
            lv_obj_set_pos(name, x, y + 2);
            style_label(name);

            if (g_bump_count + 2 > static_cast<int>(sizeof(g_bumps) / sizeof(g_bumps[0]))) return;
            g_bumps[g_bump_count] = {this, field, -1};
            make_button(parent, "-", x + 48, y, 28, 20, on_bump_event, &g_bumps[g_bump_count]);
            ++g_bump_count;

            value_labels_[field] = lv_label_create(parent);
            lv_label_set_text(value_labels_[field], "0");
            lv_obj_set_pos(value_labels_[field], x + 80, y + 2);
            style_label(value_labels_[field]);

            g_bumps[g_bump_count] = {this, field, +1};
            make_button(parent, "+", x + 148, y, 28, 20, on_bump_event, &g_bumps[g_bump_count]);
            ++g_bump_count;
        }
    };

    build_column(8, "DRIVE in", drive_fields);
    build_column(250, "TURN deg", turn_fields);

    const int actions[] = {ACT_DRIVE_POS, ACT_DRIVE_NEG, ACT_TURN_POS, ACT_TURN_NEG, ACT_PRINT, ACT_CONTROLLER,
                           ACT_POSE0};
    const char* action_text[] = {"+12in", "-12in", "+90", "-90", "Print", "Ctrl", "Pose0"};
    const int action_count = static_cast<int>(sizeof(actions) / sizeof(actions[0]));
    const lv_coord_t action_w = 64;
    const lv_coord_t action_gap = 3;
    for (int i = 0; i < action_count; ++i) {
        if (g_action_count >= static_cast<int>(sizeof(g_actions) / sizeof(g_actions[0]))) break;
        g_actions[g_action_count] = {this, actions[i]};
        lv_obj_t* button = make_button(parent, action_text[i], 6 + i * (action_w + action_gap), 196, action_w, 22,
                                       on_action_event, &g_actions[g_action_count]);
        lv_obj_set_style_pad_hor(button, 1, 0);
        lv_obj_set_style_pad_ver(button, 0, 0);
        lv_obj_t* label = lv_obj_get_child(button, 0);
        if (label != nullptr) lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        ++g_action_count;
    }

    note_ = lv_label_create(parent);
    lv_label_set_text(note_, note_text_);
    lv_obj_set_pos(note_, 8, 220);
    style_label(note_);

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
    std::snprintf(line2, 32, "A+12 B90 R1=0");
}

void PidTuner::handle_controller(pros::Controller& controller) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) &&
        !controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        controller_mode_ = false;
        drive.cancel_movement();
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
