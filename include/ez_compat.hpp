#pragma once
#include "lemlib/api.hpp"
#include "ez_compat/pid.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

namespace ez_compat {

struct PIDGains {
        double p = 0;
        double i = 0;
        double d = 0;
};

class EzDrive {
    public:
        bool isXDrive = false;

        EzDrive(lemlib::Chassis& chassis_ref, lemlib::Drivetrain drivetrain_config, pros::Motor* fl = nullptr,
                pros::Motor* bl = nullptr, pros::Motor* fr = nullptr, pros::Motor* br = nullptr);

        void set_target_heading(double angle);
        void set_drive_constants(double p, double i, double d, double start_i);
        void set_heading_constants(double p, double i, double d, double start_i);
        void set_turn_constants(double p, double i, double d, double start_i);
        void set_drive_exit_conditions(double small_err, int small_t, double big_err, int big_t, int vel_t);
        void set_turn_exit_conditions(double small_err, int small_t, double big_err, int big_t, int vel_t);
        void set_drive_velocity_threshold(double inches_per_loop);
        void set_turn_velocity_threshold(double degrees_per_loop);
        void set_heading_velocity_threshold(double degrees_per_loop);
        void set_slew_step(double step);
        void set_drive_slew(double step);
        void set_turn_slew(double step);
        void set_x_drive(bool x_drive);

        double get_drive_slew() const { return drive_slew_step; }
        double get_turn_slew() const { return turn_slew_step; }

        PID& drive_pid() { return drivePID; }
        PID& turn_pid() { return turnPID; }
        PID& heading_pid() { return headingPID; }
        const PID& drive_pid() const { return drivePID; }
        const PID& turn_pid() const { return turnPID; }
        const PID& heading_pid() const { return headingPID; }

        // distance_inches is inches from LemLib pose, not motor degrees.
        void drive(double distance_inches, int max_speed, int timeout = 1500, bool async = false);
        void drive(double distance_inches, int max_speed, int timeout, PIDGains gains, bool async = false);

        // angle is an absolute LemLib heading in degrees.
        void turn(double angle, int max_speed, int timeout = 1500, bool async = false);
        void turn(double angle, int max_speed, int timeout, PIDGains gains, bool async = false);

        void xPose(double targetX, double targetY, double targetTheta, int max_speed, int timeout, bool async = false);

        void waitUntil(double dist_inches);
        void waitUntilAngle(double degrees);

        void cancel_movement();
        // Odometry only. Does not start or cancel a motion.
        void set_pose(double x, double y, double theta);
        // Returns when the EzDrive motion task (and any chassis motion) has released
        // the drive. Also returns on a safety timeout so a stuck flag cannot hang auton.
        void waitUntilDone();
        void reset_pid();
        double get_traveled();
        bool is_busy() const;

        // Actuator entry points for LemLib motions. One tuned PID set.
        void reset_lateral_pid();
        void reset_angular_pid();
        double power_from_lateral_error(double error_inches);
        double power_from_angular_error(double error_degrees);
        bool lateral_is_settled();
        bool angular_is_settled();

        std::string constants_snippet() const;
        void print_constants() const;

    private:
        lemlib::Chassis& chassis;
        lemlib::Drivetrain drivetrain;

        pros::Motor* fl_motor;
        pros::Motor* bl_motor;
        pros::Motor* fr_motor;
        pros::Motor* br_motor;

        bool break_drive = false;
        bool break_turn = false;

        PID drivePID{0, 0, 0, 0};
        PID headingPID{0, 0, 0, 0};
        PID turnPID{0, 0, 0, 0};

        double target_heading = 0;
        double drive_slew_step = 255;
        double turn_slew_step = 255;
        double current_traveled = 0;

        double reset_x = 0;
        double reset_y = 0;
        double reset_heading = 0;

        std::atomic<int> motions_pending{0};
        int last_motion_timeout_ms = 0;

        bool begin_motion();
        void end_motion_slot();
        void stop_motors();
        void launch_motion(bool async, int timeout, const std::function<void()>& body);

        void run_drive(double distance_inches, int max_speed, int timeout);
        void run_turn(double angle, int max_speed, int timeout);
        void run_xpose(double target_x, double target_y, double target_theta, int max_speed, int timeout);
};

} // namespace ez_compat
