#include "ez_compat.hpp"
#include "ez_compat/motion_math.hpp"

#include <cmath>
#include <cstdio>

namespace ez_compat {

EzDrive::EzDrive(lemlib::Chassis& chassis_ref, lemlib::Drivetrain drivetrain_config, pros::Motor* fl, pros::Motor* bl,
                 pros::Motor* fr, pros::Motor* br)
    : chassis(chassis_ref),
      drivetrain(drivetrain_config),
      fl_motor(fl),
      bl_motor(bl),
      fr_motor(fr),
      br_motor(br) {
    target_heading = 0;
    isXDrive = false;
    current_traveled = 0;
    lemlib::Pose pose = chassis.getPose();
    reset_x = pose.x;
    reset_y = pose.y;
    reset_heading = pose.theta;
    // Per 10 ms loop. Drive is inches, turn/heading are degrees.
    drivePID.set_velocity_threshold(0.10);
    headingPID.set_velocity_threshold(0.25);
    turnPID.set_velocity_threshold(0.25);
    chassis.bindEzDrive(this);
}

void EzDrive::set_x_drive(bool x_drive) { isXDrive = x_drive; }

double EzDrive::get_traveled() {
    lemlib::Pose currentPose = chassis.getPose();
    return traveled_along_heading(currentPose.x, currentPose.y, reset_x, reset_y, reset_heading);
}

void EzDrive::set_drive_constants(double p, double i, double d, double start_i) {
    drivePID.set_constants(p, i, d, start_i);
}
void EzDrive::set_heading_constants(double p, double i, double d, double start_i) {
    headingPID.set_constants(p, i, d, start_i);
}
void EzDrive::set_turn_constants(double p, double i, double d, double start_i) {
    turnPID.set_constants(p, i, d, start_i);
}

void EzDrive::set_drive_exit_conditions(double small_err, int small_t, double big_err, int big_t, int vel_t) {
    drivePID.set_exit_conditions(small_err, small_t, big_err, big_t, vel_t);
}
void EzDrive::set_turn_exit_conditions(double small_err, int small_t, double big_err, int big_t, int vel_t) {
    turnPID.set_exit_conditions(small_err, small_t, big_err, big_t, vel_t);
}

void EzDrive::set_drive_velocity_threshold(double inches_per_loop) {
    drivePID.set_velocity_threshold(inches_per_loop);
}
void EzDrive::set_turn_velocity_threshold(double degrees_per_loop) {
    turnPID.set_velocity_threshold(degrees_per_loop);
}
void EzDrive::set_heading_velocity_threshold(double degrees_per_loop) {
    headingPID.set_velocity_threshold(degrees_per_loop);
}

void EzDrive::set_slew_step(double step) {
    drive_slew_step = step;
    turn_slew_step = step;
}
void EzDrive::set_drive_slew(double step) { drive_slew_step = step; }
void EzDrive::set_turn_slew(double step) { turn_slew_step = step; }
void EzDrive::set_target_heading(double angle) { target_heading = angle; }

bool EzDrive::begin_motion() { return chassis.acquireMotion(); }

void EzDrive::end_motion_slot() { chassis.releaseMotion(); }

void EzDrive::stop_motors() {
    if (drivetrain.leftMotors) drivetrain.leftMotors->move(0);
    if (drivetrain.rightMotors) drivetrain.rightMotors->move(0);
}

void EzDrive::launch_motion(bool async, int timeout, const std::function<void()>& body) {
    motions_pending.fetch_add(1);
    last_motion_timeout_ms = timeout;
    // Same pattern as LemLib async motions: acquire, spawn worker that re-acquires, then
    // release so the worker can take the chassis mutex.
    if (!begin_motion()) {
        motions_pending.fetch_sub(1);
        return;
    }
    chassis.setDistTraveled(0);
    if (async) {
        pros::Task task([this, body]() {
            if (!begin_motion()) {
                chassis.setDistTraveled(-1);
                motions_pending.fetch_sub(1);
                return;
            }
            break_drive = false;
            break_turn = false;
            chassis.setDistTraveled(0);
            body();
            stop_motors();
            chassis.setDistTraveled(-1);
            end_motion_slot();
            motions_pending.fetch_sub(1);
        },
                         "ez_motion");
        end_motion_slot();
        pros::delay(10);
        return;
    }
    break_drive = false;
    break_turn = false;
    body();
    stop_motors();
    chassis.setDistTraveled(-1);
    end_motion_slot();
    motions_pending.fetch_sub(1);
}

void EzDrive::run_drive(double distance_inches, int max_speed, int timeout) {
    drivePID.reset();
    headingPID.reset();
    const lemlib::Pose startPose = chassis.getPose();
    headingPID.target = startPose.theta;
    target_heading = headingPID.target;
    break_drive = false;
    drivePID.target = distance_inches;

    const std::uint32_t start_time = pros::millis();
    double last_l = 0;
    double last_r = 0;
    while (chassis.isInMotion()) {
        const lemlib::Pose currentPose = chassis.getPose();
        const double local_traveled = traveled_along_heading(currentPose.x, currentPose.y, startPose.x, startPose.y,
                                                             startPose.theta);
        current_traveled = local_traveled;
        chassis.setDistTraveled(static_cast<float>(std::fabs(local_traveled)));

        double drive_out = drivePID.compute(local_traveled);
        double heading_out = headingPID.compute(currentPose.theta, true);
        if (!std::isfinite(drive_out)) drive_out = 0;
        if (!std::isfinite(heading_out)) heading_out = 0;

        const double target_l = drive_out + heading_out;
        const double target_r = drive_out - heading_out;
        double l_power = clamp_value(target_l, last_l - drive_slew_step, last_l + drive_slew_step);
        double r_power = clamp_value(target_r, last_r - drive_slew_step, last_r + drive_slew_step);
        l_power = clamp_value(l_power, -max_speed, max_speed);
        r_power = clamp_value(r_power, -max_speed, max_speed);

        drivetrain.leftMotors->move(l_power);
        drivetrain.rightMotors->move(r_power);
        last_l = l_power;
        last_r = r_power;

        const bool timed_out = static_cast<int>(pros::millis() - start_time) > timeout;
        if (break_drive || drivePID.is_settled(10) || timed_out) break;
        pros::delay(10);
    }
    break_drive = false;
}

void EzDrive::drive(double distance_inches, int max_speed, int timeout, bool async) {
    launch_motion(async, timeout, [=, this]() { run_drive(distance_inches, max_speed, timeout); });
}

void EzDrive::drive(double distance_inches, int max_speed, int timeout, PIDGains gains, bool async) {
    const double prev_p = drivePID.kp;
    const double prev_i = drivePID.ki;
    const double prev_d = drivePID.kd;
    const double prev_s = drivePID.start_i;
    launch_motion(async, timeout, [=, this]() {
        drivePID.set_constants(gains.p, gains.i, gains.d, prev_s);
        run_drive(distance_inches, max_speed, timeout);
        drivePID.set_constants(prev_p, prev_i, prev_d, prev_s);
    });
}

void EzDrive::run_turn(double angle, int max_speed, int timeout) {
    turnPID.reset();
    turnPID.target = angle;
    target_heading = angle;
    break_turn = false;
    const std::uint32_t start_time = pros::millis();
    const double start_theta = chassis.getPose().theta;
    double last_l = 0;
    double last_r = 0;
    while (chassis.isInMotion()) {
        const double theta = chassis.getPose().theta;
        chassis.setDistTraveled(static_cast<float>(std::fabs(heading_error_deg(theta, start_theta))));

        double output = turnPID.compute(theta, true);
        if (!std::isfinite(output)) output = 0;
        double l_power = clamp_value(output, last_l - turn_slew_step, last_l + turn_slew_step);
        double r_power = clamp_value(-output, last_r - turn_slew_step, last_r + turn_slew_step);
        l_power = clamp_value(l_power, -max_speed, max_speed);
        r_power = clamp_value(r_power, -max_speed, max_speed);

        drivetrain.leftMotors->move(l_power);
        drivetrain.rightMotors->move(r_power);
        last_l = l_power;
        last_r = r_power;

        const bool timed_out = static_cast<int>(pros::millis() - start_time) > timeout;
        if (break_turn || turnPID.is_settled(10) || timed_out) break;
        pros::delay(10);
    }
    break_turn = false;
}

void EzDrive::turn(double angle, int max_speed, int timeout, bool async) {
    launch_motion(async, timeout, [=, this]() { run_turn(angle, max_speed, timeout); });
}

void EzDrive::turn(double angle, int max_speed, int timeout, PIDGains gains, bool async) {
    const double prev_p = turnPID.kp;
    const double prev_i = turnPID.ki;
    const double prev_d = turnPID.kd;
    const double prev_s = turnPID.start_i;
    launch_motion(async, timeout, [=, this]() {
        turnPID.set_constants(gains.p, gains.i, gains.d, prev_s);
        run_turn(angle, max_speed, timeout);
        turnPID.set_constants(prev_p, prev_i, prev_d, prev_s);
    });
}

void EzDrive::run_xpose(double target_x, double target_y, double target_theta, int max_speed, int timeout) {
    drivePID.reset();
    turnPID.reset();
    break_drive = false;
    drivePID.target = 0;
    turnPID.target = target_theta;
    const std::uint32_t start_time = pros::millis();
    double last_fl = 0, last_fr = 0, last_bl = 0, last_br = 0;

    while (chassis.isInMotion()) {
        const lemlib::Pose pose = chassis.getPose();
        const double dx = target_x - pose.x;
        const double dy = target_y - pose.y;
        const double drive_error = std::hypot(dx, dy);
        chassis.setDistTraveled(static_cast<float>(drive_error));

        double drive_out = drivePID.compute(-drive_error);
        double turn_out = turnPID.compute(pose.theta, true);
        if (!std::isfinite(drive_out)) drive_out = 0;
        if (!std::isfinite(turn_out)) turn_out = 0;

        const double target_heading_rad = std::atan2(dy, dx);
        const double current_heading_rad = pose.theta * M_PI / 180.0;
        const double alpha = target_heading_rad - current_heading_rad;

        const double t_fl = drive_out * std::cos(alpha - M_PI / 4) + turn_out;
        const double t_fr = drive_out * std::sin(alpha - M_PI / 4) - turn_out;
        const double t_bl = drive_out * std::sin(alpha - M_PI / 4) + turn_out;
        const double t_br = drive_out * std::cos(alpha - M_PI / 4) - turn_out;

        double fl = clamp_value(t_fl, last_fl - drive_slew_step, last_fl + drive_slew_step);
        double fr = clamp_value(t_fr, last_fr - drive_slew_step, last_fr + drive_slew_step);
        double bl = clamp_value(t_bl, last_bl - drive_slew_step, last_bl + drive_slew_step);
        double br = clamp_value(t_br, last_br - drive_slew_step, last_br + drive_slew_step);
        fl = clamp_value(fl, -max_speed, max_speed);
        fr = clamp_value(fr, -max_speed, max_speed);
        bl = clamp_value(bl, -max_speed, max_speed);
        br = clamp_value(br, -max_speed, max_speed);

        if (fl_motor) fl_motor->move(fl);
        if (bl_motor) bl_motor->move(bl);
        if (fr_motor) fr_motor->move(fr);
        if (br_motor) br_motor->move(br);
        last_fl = fl;
        last_fr = fr;
        last_bl = bl;
        last_br = br;

        const bool drive_settled = drivePID.is_settled(10);
        const bool turn_settled = turnPID.is_settled(10);
        const bool timed_out = static_cast<int>(pros::millis() - start_time) > timeout;
        if (break_drive || (drive_settled && turn_settled) || timed_out) break;
        pros::delay(10);
    }
    break_drive = false;
}

void EzDrive::xPose(double targetX, double targetY, double targetTheta, int max_speed, int timeout, bool async) {
    if (!isXDrive) return;
    launch_motion(async, timeout,
                  [=, this]() { run_xpose(targetX, targetY, targetTheta, max_speed, timeout); });
}

void EzDrive::waitUntil(double dist_inches) {
    const lemlib::Pose startPose = chassis.getPose();
    const double target_dist = std::fabs(dist_inches);
    while (true) {
        const lemlib::Pose currentPose = chassis.getPose();
        const double traveled = std::hypot(currentPose.x - startPose.x, currentPose.y - startPose.y);
        if (traveled >= target_dist || break_drive) break;
        pros::delay(10);
    }
}

void EzDrive::waitUntilAngle(double degrees) {
    const lemlib::Pose startPose = chassis.getPose();
    const double target_angle = std::fabs(degrees);
    while (true) {
        const lemlib::Pose currentPose = chassis.getPose();
        const double turned = std::fabs(heading_error_deg(currentPose.theta, startPose.theta));
        if (turned >= target_angle || break_turn) break;
        pros::delay(10);
    }
}

void EzDrive::waitUntilDone() {
    const std::uint32_t start = pros::millis();
    int cap = last_motion_timeout_ms > 0 ? last_motion_timeout_ms + 400 : 20000;
    if (cap < 400) cap = 400;
    while (motions_pending.load() > 0 || chassis.isInMotion()) {
        if (static_cast<int>(pros::millis() - start) > cap) {
            std::printf("EzDrive::waitUntilDone safety timeout (%d ms)\n", cap);
            cancel_movement();
            break;
        }
        pros::delay(10);
    }
}

void EzDrive::cancel_movement() {
    break_drive = true;
    break_turn = true;
    chassis.cancelAllMotions();
    pros::delay(20);
}

void EzDrive::set_pose(double x, double y, double theta) {
    chassis.setPose(static_cast<float>(x), static_cast<float>(y), static_cast<float>(theta));
}

void EzDrive::reset_pid() {
    drivePID.reset();
    headingPID.reset();
    turnPID.reset();
    current_traveled = 0;
    const lemlib::Pose pose = chassis.getPose();
    reset_x = pose.x;
    reset_y = pose.y;
    reset_heading = pose.theta;
}

bool EzDrive::is_busy() const { return motions_pending.load() > 0 || chassis.isInMotion(); }

void EzDrive::reset_lateral_pid() { drivePID.reset(); }
void EzDrive::reset_angular_pid() { turnPID.reset(); }

double EzDrive::power_from_lateral_error(double error_inches) { return drivePID.compute_error(error_inches, false); }

double EzDrive::power_from_angular_error(double error_degrees) { return turnPID.compute_error(error_degrees, true); }

bool EzDrive::lateral_is_settled() { return drivePID.is_settled(10); }
bool EzDrive::angular_is_settled() { return turnPID.is_settled(10); }

std::string EzDrive::constants_snippet() const {
    char buf[900];
    std::snprintf(buf, sizeof(buf),
                  "ezPID.set_drive_constants(%.4f, %.4f, %.4f, %.4f);\n"
                  "ezPID.set_heading_constants(%.4f, %.4f, %.4f, %.4f);\n"
                  "ezPID.set_turn_constants(%.4f, %.4f, %.4f, %.4f);\n"
                  "ezPID.set_drive_exit_conditions(%.3f, %d, %.3f, %d, %d); // inches, ms\n"
                  "ezPID.set_turn_exit_conditions(%.3f, %d, %.3f, %d, %d); // degrees, ms\n"
                  "ezPID.set_drive_velocity_threshold(%.4f); // inches per 10ms\n"
                  "ezPID.set_turn_velocity_threshold(%.4f); // degrees per 10ms\n"
                  "ezPID.set_heading_velocity_threshold(%.4f); // degrees per 10ms\n"
                  "ezPID.set_drive_slew(%.2f);\n"
                  "ezPID.set_turn_slew(%.2f);\n",
                  drivePID.kp, drivePID.ki, drivePID.kd, drivePID.start_i, headingPID.kp, headingPID.ki, headingPID.kd,
                  headingPID.start_i, turnPID.kp, turnPID.ki, turnPID.kd, turnPID.start_i, drivePID.small_error,
                  drivePID.small_time, drivePID.big_error, drivePID.big_time, drivePID.velocity_time,
                  turnPID.small_error, turnPID.small_time, turnPID.big_error, turnPID.big_time, turnPID.velocity_time,
                  drivePID.velocity_threshold, turnPID.velocity_threshold, headingPID.velocity_threshold,
                  drive_slew_step, turn_slew_step);
    return std::string(buf);
}

void EzDrive::print_constants() const { std::printf("%s", constants_snippet().c_str()); }

} // namespace ez_compat
