#pragma once

#include "ezlib/ez_compat.hpp"
#include "pros/misc.hpp"
#include "liblvgl/lvgl.h"

namespace ez_compat {

// Brain + controller tuner for EzDrive gains.
// Logs to /usd/pid_tune_log.csv; L2/R2 write /usd/findingOffset.csv (CCW/CW x10).
// Print → USB (+ SD CSV). Dash/PID coexist under malloc LVGL.
class PidTuner {
    public:
        explicit PidTuner(EzDrive& drive) : drive(drive) {}

        void start();
        void focus(); // show PID overlay
        void hide();  // hide PID overlay
        bool alive() const { return page_ != nullptr; }
        bool controller_mode() const { return controller_mode_; }
        void toggle_controller_mode();
        // Stop controller mode / offset spins (keeps PID page alive).
        void force_off();
        // Match mode / teardown: force_off + delete PID overlay.
        // Call start() again after leaving field control to recreate.
        void release_ui();
        void handle_controller(pros::Controller& controller);
        // True while the tuner owns the sticks, or any Ez/chassis motion is running.
        bool suppress_arcade() const;
        void format_controller(char line0[32], char line1[32], char line2[32]) const;

    private:
        void build_ui(lv_obj_t* parent);
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
        bool offset_running_ = false;
        int field_ = 0;
        lv_obj_t* page_ = nullptr; // fullscreen overlay on active screen
        lv_timer_t* timer_ = nullptr;
        lv_obj_t* field_label_ = nullptr;
        lv_obj_t* value_label_ = nullptr;
        lv_obj_t* live_ = nullptr;
        lv_obj_t* note_ = nullptr;
        char note_text_[96] = "X+A enter  L2=left10  R2=right10  <> resetpose";
        bool note_dirty_ = false;

        // Last completed drive sample (shown on brain).
        double last_imu_delta_ = 0;
        double last_odom_delta_ = 0;
        double last_imu_heading_ = 0;
        double last_odom_theta_ = 0;
        bool has_drive_sample_ = false;

        void refresh();
        void bump(int field, int direction);
        void step_field(int direction);
        void print_now();
        void test_drive(double inches);
        void test_turn(double degrees);
        void zero_pose();
        // direction: +1 = CCW/left (+90°×40), -1 = CW/right (−90°×40).
        void run_offset_spins(int direction);
        void log_pid_event(const char* event, double cmd_inches, double cmd_heading, double start_odom_theta,
                           double start_imu_rot, double start_imu_heading);
        // Sample every ~15ms while drive/turn runs (oscillation / settle curve).
        void log_pid_run(const char* event_prefix, double cmd_inches, double cmd_heading, double start_odom_theta,
                         double start_imu_rot, double start_imu_heading);
        void set_note(const char* text);
        const char* field_name(int field) const;
        double field_value(int field) const;
        void set_field_value(int field, double value);
        double field_step(int field) const;

        static void on_bump_event(lv_event_t* event);
        static void on_action_event(lv_event_t* event);
        static void on_timer(lv_timer_t* timer);
};

} // namespace ez_compat
