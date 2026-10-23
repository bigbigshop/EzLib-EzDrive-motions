// Host tests for EzDrive PID, settle, and chassis motion math.
// No PROS, no robot. Each case function is invoked three times.

#include "ez_compat/motion_math.hpp"
#include "ez_compat/pid.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

int g_fails = 0;

void check(bool cond, const char* expr, const char* file, int line) {
    if (cond) return;
    std::printf("FAIL %s:%d  %s\n", file, line, expr);
    ++g_fails;
}

#define CHECK(cond) check(static_cast<bool>(cond), #cond, __FILE__, __LINE__)

void repeat3(const char* name, void (*fn)(int)) {
    for (int run = 0; run < 3; ++run) {
        std::printf("RUN %s #%d\n", name, run + 1);
        fn(run);
    }
}

bool finite_num(double value) { return std::isfinite(value); }

struct Plant {
        double x = 0;
        double y = 0;
        double theta_deg = 0; // LemLib heading: 0 faces +Y, clockwise positive
        double wheel_l = 0;   // in/s, lagged so kD sees a real derivative
        double wheel_r = 0;
        double track = 11.45;
        double max_ips = 360.0 / 60.0 * M_PI * 3.25;

        void apply(double left, double right, double dt) {
            const double target_l = (left / 127.0) * max_ips;
            const double target_r = (right / 127.0) * max_ips;
            const double max_accel = 70.0 * dt; // in/s per step
            wheel_l += ez_compat::clamp_value(target_l - wheel_l, -max_accel, max_accel);
            wheel_r += ez_compat::clamp_value(target_r - wheel_r, -max_accel, max_accel);
            const double v = 0.5 * (wheel_l + wheel_r);
            const double rad = theta_deg * M_PI / 180.0;
            x += v * dt * std::sin(rad);
            y += v * dt * std::cos(rad);
            const double omega = (wheel_l - wheel_r) / track;
            theta_deg += omega * dt * 180.0 / M_PI;
        }
};

ez_compat::PID make_drive_pid() {
    ez_compat::PID pid(8.25, 0, 42, 0);
    pid.set_exit_conditions(0.5, 100, 2.0, 500, 0);
    pid.set_velocity_threshold(0.10);
    return pid;
}

ez_compat::PID make_turn_pid() {
    ez_compat::PID pid(4.1, 0, 30, 15);
    pid.set_exit_conditions(1.0, 100, 3.0, 250, 0);
    pid.set_velocity_threshold(0.25);
    return pid;
}

ez_compat::PID make_heading_pid() {
    ez_compat::PID pid(4, 0, 10, 0);
    pid.set_exit_conditions(1.0, 100, 3.0, 250, 0);
    pid.set_velocity_threshold(0.25);
    return pid;
}

void test_settled_not_derivative_only(int run) {
    const double errors[] = {12.0, 40.0, -25.0};
    ez_compat::PID pid = make_drive_pid();
    pid.target = errors[run];
    pid.velocity_time = 500;
    bool settled = false;
    for (int i = 0; i < 200; ++i) {
        const double output = pid.compute(0.0);
        CHECK(finite_num(output));
        if (pid.is_settled(10)) settled = true;
    }
    CHECK(!settled);
    CHECK(std::fabs(pid.error - errors[run]) < 1e-6);
    CHECK(std::fabs(pid.derivative) <= pid.velocity_threshold);
}

void test_settled_in_band_when_slow(int run) {
    const double targets[] = {10.0, 24.0, -12.0};
    ez_compat::PID pid = make_drive_pid();
    pid.target = targets[run];
    const double measurement = targets[run] - 0.2;
    pid.compute(0.0);
    CHECK(!pid.is_settled(10));
    bool settled = false;
    for (int i = 0; i < 20; ++i) {
        const double output = pid.compute(measurement);
        CHECK(finite_num(output));
        if (pid.is_settled(10)) settled = true;
    }
    CHECK(settled);
    CHECK(std::fabs(pid.error) <= pid.small_error);
}

void test_settled_rejects_fast_inside_band(int run) {
    ez_compat::PID pid = make_drive_pid();
    pid.target = 10.0;
    pid.compute(9.7);
    bool settled = false;
    double measurement = 9.7;
    const double step = 0.4 + 0.05 * run; // faster than 0.10 in per sample
    for (int i = 0; i < 40; ++i) {
        measurement += (run == 2) ? -step : step;
        const double output = pid.compute(measurement);
        CHECK(finite_num(output));
        if (std::fabs(pid.error) <= pid.small_error && pid.is_settled(10)) settled = true;
    }
    CHECK(!settled);
}

void test_drive_command_and_settle(int run) {
    const double targets[] = {12.0, 24.0, -18.0};
    const double target = targets[run];
    ez_compat::PID drive = make_drive_pid();
    ez_compat::PID heading = make_heading_pid();
    drive.target = target;
    heading.target = 0;
    Plant bot;
    double last_l = 0;
    double last_r = 0;
    bool settled = false;
    double sign_sample = 0;
    for (int step = 0; step < 600; ++step) {
        const double traveled = ez_compat::traveled_along_heading(bot.x, bot.y, 0, 0, 0);
        const double drive_out = drive.compute(traveled);
        const double head_out = heading.compute(bot.theta_deg, true);
        CHECK(finite_num(drive_out));
        CHECK(finite_num(head_out));
        CHECK(finite_num(bot.x));
        CHECK(finite_num(bot.y));
        CHECK(finite_num(bot.theta_deg));
        const double raw_l = drive_out + head_out;
        const double raw_r = drive_out - head_out;
        const double l = ez_compat::clamp_value(ez_compat::slew_towards(raw_l, last_l, 12), -100, 100);
        const double r = ez_compat::clamp_value(ez_compat::slew_towards(raw_r, last_r, 12), -100, 100);
        if (step == 4) sign_sample = l;
        last_l = l;
        last_r = r;
        bot.apply(l, r, 0.01);
        if (drive.is_settled(10)) {
            settled = true;
            break;
        }
    }
    CHECK(settled);
    CHECK(target > 0 ? sign_sample > 0 : sign_sample < 0);
    const double traveled = ez_compat::traveled_along_heading(bot.x, bot.y, 0, 0, 0);
    CHECK(std::fabs(traveled - target) <= 2.0);
    CHECK(finite_num(traveled));
}

void test_turn_command_and_settle(int run) {
    const double targets[] = {90.0, -60.0, 180.0};
    const double target = targets[run];
    ez_compat::PID turn = make_turn_pid();
    turn.target = target;
    Plant bot;
    double last_l = 0;
    double last_r = 0;
    bool settled = false;
    double sign_sample = 0;
    for (int step = 0; step < 700; ++step) {
        const double output = turn.compute(bot.theta_deg, true);
        CHECK(finite_num(output));
        const double l = ez_compat::clamp_value(ez_compat::slew_towards(output, last_l, 255), -90, 90);
        const double r = ez_compat::clamp_value(ez_compat::slew_towards(-output, last_r, 255), -90, 90);
        if (step == 2) sign_sample = l;
        last_l = l;
        last_r = r;
        bot.apply(l, r, 0.01);
        CHECK(finite_num(bot.theta_deg));
        if (turn.is_settled(10)) {
            settled = true;
            break;
        }
    }
    CHECK(settled);
    const double wrapped = ez_compat::heading_error_deg(target, 0);
    CHECK(wrapped > 0 ? sign_sample > 0 : sign_sample < 0);
    CHECK(std::fabs(ez_compat::heading_error_deg(target, bot.theta_deg)) <= 3.0);
}

void test_turn_to_heading(int run) {
    const double current[] = {0.0, 10.0, 0.0};
    const double target[] = {90.0, 350.0, 180.0};
    const double expect[] = {90.0, -20.0, 180.0};
    const double error = ez_compat::heading_error_deg(target[run], current[run]);
    CHECK(std::fabs(error - expect[run]) < 1e-6);
    ez_compat::PID turn = make_turn_pid();
    const double power = turn.compute_error(error, true);
    CHECK(finite_num(power));
    CHECK(error > 0 ? power > 0 : (error < 0 ? power < 0 : power == 0));
    const ez_compat::WheelPower wheels = ez_compat::mix_arcade(0, power, 110);
    CHECK(wheels.finite);
    if (power > 0) {
        CHECK(wheels.left > 0);
        CHECK(wheels.right < 0);
    } else if (power < 0) {
        CHECK(wheels.left < 0);
        CHECK(wheels.right > 0);
    }
    // Directional long-way error stays finite and keeps its forced sign.
    const double cw = ez_compat::heading_error_deg(350, 10, ez_compat::TurnDir::CW);
    CHECK(cw > 0);
    CHECK(finite_num(cw));
}

void test_swing(int run) {
    const double errors[] = {30.0, -20.0, 45.0};
    const bool lock_left[] = {true, true, false};
    ez_compat::PID turn = make_turn_pid();
    const double power = turn.compute_error(errors[run], true);
    CHECK(finite_num(power));
    CHECK(errors[run] > 0 ? power > 0 : power < 0);
    const ez_compat::SwingPower swing = ez_compat::swing_power(power, lock_left[run]);
    CHECK(swing.finite);
    if (lock_left[run]) {
        CHECK(swing.left == 0);
        CHECK(swing.right == -power);
    } else {
        CHECK(swing.right == 0);
        CHECK(swing.left == power);
    }
    CHECK(finite_num(swing.left));
    CHECK(finite_num(swing.right));
}

void test_move_to_point(int run) {
    struct Case {
            double x;
            double y;
            bool expect_forward;
    };
    const Case cases[] = {{0, 24, true}, {24, 0, false}, {0, -24, false}};
    const Case c = cases[run];
    // Standard pose facing +Y.
    const ez_compat::Pose2 pose{0, 0, M_PI_2};
    const ez_compat::PointErrors err = ez_compat::move_to_point_errors(pose, {c.x, c.y, 0}, true);
    CHECK(err.finite);
    CHECK(finite_num(err.lateral_in));
    CHECK(finite_num(err.angular_deg));
    if (run == 0) {
        CHECK(std::fabs(err.lateral_in - 24.0) < 1e-6);
        CHECK(std::fabs(err.angular_deg) < 1e-6);
    } else {
        CHECK(std::fabs(err.angular_deg) > 10.0);
    }

    ez_compat::PID drive = make_drive_pid();
    ez_compat::PID turn = make_turn_pid();
    const double lateral = drive.compute_error(err.lateral_in, false);
    const double angular = turn.compute_error(err.angular_deg, true);
    CHECK(finite_num(lateral));
    CHECK(finite_num(angular));
    if (err.lateral_in > 0.5) CHECK(lateral > 0);
    if (err.lateral_in < -0.5) CHECK(lateral < 0);
    const ez_compat::WheelPower wheels = ez_compat::mix_arcade(lateral, angular, 100);
    CHECK(wheels.finite);
    if (c.expect_forward) {
        CHECK(wheels.left > 0);
        CHECK(wheels.right > 0);
    }

    if (run != 0) return;

    Plant bot;
    drive.reset();
    turn.reset();
    double last_l = 0;
    double last_r = 0;
    bool settled = false;
    for (int step = 0; step < 800; ++step) {
        const double theta_std = M_PI_2 - bot.theta_deg * M_PI / 180.0;
        const ez_compat::PointErrors live =
            ez_compat::move_to_point_errors({bot.x, bot.y, theta_std}, {0, 24, 0}, true);
        CHECK(live.finite);
        const bool close = live.distance_in < 7.5;
        const double lat = drive.compute_error(live.lateral_in, false);
        const double ang = close ? 0.0 : turn.compute_error(live.angular_deg, true);
        const ez_compat::WheelPower cmd = ez_compat::mix_arcade(lat, ang, 100);
        CHECK(cmd.finite);
        const double l = ez_compat::clamp_value(ez_compat::slew_towards(cmd.left, last_l, 12), -100, 100);
        const double r = ez_compat::clamp_value(ez_compat::slew_towards(cmd.right, last_r, 12), -100, 100);
        last_l = l;
        last_r = r;
        bot.apply(l, r, 0.01);
        CHECK(finite_num(bot.x));
        CHECK(finite_num(bot.y));
        const bool timed_out = step * 10 > 7000;
        if (close && drive.is_settled(10)) {
            settled = true;
            break;
        }
        if (timed_out) break;
    }
    CHECK(settled);
    CHECK(std::hypot(bot.x, bot.y - 24.0) <= 2.0);
}

void test_move_to_pose(int run) {
    struct Case {
            double x;
            double y;
            double heading_deg;
    };
    // LemLib heading 0 faces +Y. Standard heading = pi/2 - rad(heading).
    const Case cases[] = {{0, 30, 0}, {20, 20, 45}, {10, 0, 90}};
    const Case c = cases[run];
    const double target_std = M_PI_2 - c.heading_deg * M_PI / 180.0;
    const ez_compat::Pose2 start{0, 0, M_PI_2};
    const ez_compat::PoseErrors err =
        ez_compat::move_to_pose_errors(start, {c.x, c.y, target_std}, true, 0.6, false);
    CHECK(err.finite);
    CHECK(finite_num(err.carrot_x));
    CHECK(finite_num(err.carrot_y));
    CHECK(finite_num(err.lateral_in));
    CHECK(finite_num(err.angular_deg));

    ez_compat::PID drive = make_drive_pid();
    ez_compat::PID turn = make_turn_pid();
    const double lateral = drive.compute_error(err.lateral_in, false);
    const double angular = turn.compute_error(err.angular_deg, true);
    const ez_compat::WheelPower wheels = ez_compat::mix_arcade(lateral, angular, 100);
    CHECK(wheels.finite);
    CHECK(finite_num(lateral));
    CHECK(finite_num(angular));
    if (run == 0) {
        CHECK(lateral > 0);
        CHECK(wheels.left > 0);
        CHECK(wheels.right > 0);
    }

    Plant bot;
    drive.reset();
    turn.reset();
    double last_lat = 0;
    bool settled = false;
    for (int step = 0; step < 900; ++step) {
        const double theta_std = M_PI_2 - bot.theta_deg * M_PI / 180.0;
        const double dist = std::hypot(c.x - bot.x, c.y - bot.y);
        const bool close = dist < 7.5;
        const ez_compat::PoseErrors live = ez_compat::move_to_pose_errors(
            {bot.x, bot.y, theta_std}, {c.x, c.y, target_std}, true, 0.6, close);
        CHECK(live.finite);
        const double lat = drive.compute_error(live.lateral_in, false);
        const double ang = turn.compute_error(live.angular_deg, true);
        double lat_cmd = lat;
        double ang_cmd = ang;
        lat_cmd = ez_compat::clamp_value(lat_cmd, -100, 100);
        ang_cmd = ez_compat::clamp_value(ang_cmd, -100, 100);
        if (!close) lat_cmd = ez_compat::slew_towards(lat_cmd, last_lat, 12);
        last_lat = lat_cmd;
        const ez_compat::WheelPower cmd = ez_compat::mix_arcade(lat_cmd, ang_cmd, 100);
        CHECK(cmd.finite);
        bot.apply(cmd.left, cmd.right, 0.01);
        const bool both = drive.is_settled(10) && turn.is_settled(10);
        if (close && both) {
            settled = true;
            break;
        }
    }
    if (run == 0) {
        CHECK(settled);
        CHECK(std::hypot(bot.x - c.x, bot.y - c.y) <= 2.5);
    } else {
        CHECK(finite_num(bot.x));
        CHECK(finite_num(bot.y));
        CHECK(finite_num(bot.theta_deg));
    }
}

void test_traveled_projection(int run) {
    if (run == 0) {
        CHECK(std::fabs(ez_compat::traveled_along_heading(0, 10, 0, 0, 0) - 10) < 1e-6);
    } else if (run == 1) {
        CHECK(std::fabs(ez_compat::traveled_along_heading(10, 0, 0, 0, 90) - 10) < 1e-6);
    } else {
        CHECK(std::fabs(ez_compat::traveled_along_heading(0, -5, 0, 0, 0) + 5) < 1e-6);
    }
}

} // namespace

int main() {
    repeat3("settled_not_derivative_only", test_settled_not_derivative_only);
    repeat3("settled_in_band_when_slow", test_settled_in_band_when_slow);
    repeat3("settled_rejects_fast_inside_band", test_settled_rejects_fast_inside_band);
    repeat3("drive", test_drive_command_and_settle);
    repeat3("turn", test_turn_command_and_settle);
    repeat3("turnToHeading", test_turn_to_heading);
    repeat3("swing", test_swing);
    repeat3("moveToPoint", test_move_to_point);
    repeat3("moveToPose", test_move_to_pose);
    repeat3("traveled_along_heading", test_traveled_projection);

    if (g_fails != 0) {
        std::printf("%d assertion(s) failed\n", g_fails);
        return 1;
    }
    std::printf("all motion tests passed (3 runs each)\n");
    return 0;
}
