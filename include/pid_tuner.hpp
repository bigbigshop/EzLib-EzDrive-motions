#pragma once

#include "ez_compat.hpp"
#include "pros/misc.hpp"
#include "liblvgl/lvgl.h"
#include "robodash/api.h"

namespace ez_compat {

// Brain + controller tuner for EzDrive gains. Nothing is written to the SD card.
class PidTuner {
    public:
        explicit PidTuner(EzDrive& drive) : drive(drive) {}

        void start();
        bool controller_mode() const { return controller_mode_; }
        void toggle_controller_mode();
        void handle_controller(pros::Controller& controller);
        // True while the tuner owns the sticks, or any Ez/chassis motion is running.
        bool suppress_arcade() const;
        void format_controller(char line0[32], char line1[32], char line2[32]) const;

    private:
        enum Field : int {
            F_DRIVE_P = 0,
            F_DRIVE_I,
            F_DRIVE_D,
            F_DRIVE_SMALL,
            F_DRIVE_BIG,
            F_DRIVE_VEL,
            F_TURN_P,
            F_TURN_I,
            F_TURN_D,
            F_TURN_SMALL,
            F_TURN_BIG,
            F_TURN_VEL,
            F_HEAD_P,
            F_HEAD_I,
            F_HEAD_D,
            F_COUNT
        };

        EzDrive& drive;
        bool controller_mode_ = false;
        bool started_ = false;
        int field_ = 0;
        rd_view_t* view_ = nullptr;
        lv_obj_t* value_labels_[F_TURN_VEL + 1] = {};
        lv_obj_t* live_ = nullptr;
        lv_obj_t* note_ = nullptr;
        char note_text_[96] = "Print copies constants to the terminal";
        bool note_dirty_ = false;

        void refresh();
        void bump(int field, int direction);
        void print_now();
        void test_drive(double inches);
        void test_turn(double degrees);
        const char* field_name(int field) const;
        double field_value(int field) const;
        void set_field_value(int field, double value);
        double field_step(int field) const;

        static void on_bump_event(lv_event_t* event);
        static void on_action_event(lv_event_t* event);
        static void on_timer(lv_timer_t* timer);
};

} // namespace ez_compat
