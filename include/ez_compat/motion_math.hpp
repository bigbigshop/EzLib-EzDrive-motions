#pragma once

// Pros-free geometry shared by chassis motions and the host tests.
// Positions are inches. Headings passed to heading_error_deg() are degrees.
// Poses passed to move_to_point / move_to_pose are LemLib "standard" radians
// (0 along +X, counter-clockwise), matching Chassis::getPose(true, true).

#include <cmath>

namespace ez_compat {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum class TurnDir { Auto, CW, CCW };

inline double sanitize_deg(double angle) {
    return std::fmod(std::fmod(angle, 360.0) + 360.0, 360.0);
}

// Same contract as lemlib::angleError(..., radians = false).
inline double heading_error_deg(double target_deg, double position_deg, TurnDir direction = TurnDir::Auto) {
    const double target = sanitize_deg(target_deg);
    const double position = sanitize_deg(position_deg);
    const double raw = target - position;
    switch (direction) {
        case TurnDir::CW: return raw < 0.0 ? raw + 360.0 : raw;
        case TurnDir::CCW: return raw > 0.0 ? raw - 360.0 : raw;
        default: return std::remainder(raw, 360.0);
    }
}

inline double angle_error_rad(double target, double position) {
    return std::remainder(target - position, 2.0 * M_PI);
}

struct Pose2 {
        double x = 0;
        double y = 0;
        double theta = 0;
};

inline double pose_distance(Pose2 a, Pose2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

inline double pose_angle(Pose2 from, Pose2 to) { return std::atan2(to.y - from.y, to.x - from.x); }

struct PointErrors {
        double lateral_in = 0;
        double angular_rad = 0;
        double angular_deg = 0;
        double distance_in = 0;
        bool finite = false;
};

// forwards == false faces the point with the back of the robot.
inline PointErrors move_to_point_errors(Pose2 pose, Pose2 target, bool forwards) {
    PointErrors out;
    out.distance_in = pose_distance(pose, target);
    const double angle_to = pose_angle(pose, target);
    const double robot = forwards ? pose.theta : pose.theta + M_PI;
    out.angular_rad = angle_error_rad(robot, angle_to);
    out.angular_deg = out.angular_rad * 180.0 / M_PI;
    const double facing = angle_error_rad(pose.theta, angle_to);
    out.lateral_in = out.distance_in * std::cos(facing);
    out.finite = std::isfinite(out.lateral_in) && std::isfinite(out.angular_rad) && std::isfinite(out.distance_in);
    return out;
}

struct PoseErrors {
        double lateral_in = 0;
        double angular_rad = 0;
        double angular_deg = 0;
        double carrot_x = 0;
        double carrot_y = 0;
        double distance_in = 0;
        bool finite = false;
};

inline double sgn_nonzero(double value) { return value < 0.0 ? -1.0 : 1.0; }

// target.theta is the desired heading already converted to standard radians,
// including the backwards flip the motion applies once before the loop.
inline PoseErrors move_to_pose_errors(Pose2 pose, Pose2 target, bool forwards, double lead, bool close) {
    PoseErrors out;
    out.distance_in = pose_distance(pose, target);
    double carrot_x = target.x - std::cos(target.theta) * lead * out.distance_in;
    double carrot_y = target.y - std::sin(target.theta) * lead * out.distance_in;
    if (close) {
        carrot_x = target.x;
        carrot_y = target.y;
    }
    out.carrot_x = carrot_x;
    out.carrot_y = carrot_y;

    Pose2 carrot{carrot_x, carrot_y, 0};
    const double dist_carrot = pose_distance(pose, carrot);
    const double angle_to_carrot = pose_angle(pose, carrot);
    const double robot = forwards ? pose.theta : pose.theta + M_PI;
    out.angular_rad = close ? angle_error_rad(robot, target.theta) : angle_error_rad(robot, angle_to_carrot);
    out.angular_deg = out.angular_rad * 180.0 / M_PI;
    const double face = angle_error_rad(pose.theta, angle_to_carrot);
    if (close) out.lateral_in = dist_carrot * std::cos(face);
    else out.lateral_in = dist_carrot * sgn_nonzero(std::cos(face));
    out.finite = std::isfinite(out.lateral_in) && std::isfinite(out.angular_rad) && std::isfinite(carrot_x) &&
                 std::isfinite(carrot_y);
    return out;
}

struct WheelPower {
        double left = 0;
        double right = 0;
        bool finite = false;
};

inline WheelPower mix_arcade(double lateral, double angular, double max_speed) {
    WheelPower out;
    out.left = lateral + angular;
    out.right = lateral - angular;
    if (max_speed > 0.0) {
        const double ratio = std::max(std::fabs(out.left), std::fabs(out.right)) / max_speed;
        if (ratio > 1.0) {
            out.left /= ratio;
            out.right /= ratio;
        }
    }
    out.finite = std::isfinite(out.left) && std::isfinite(out.right);
    return out;
}

struct SwingPower {
        double left = 0;
        double right = 0;
        bool left_locked = false;
        bool finite = false;
};

// lock_left mirrors Chassis::swing* with DriveSide::LEFT.
inline SwingPower swing_power(double angular, bool lock_left) {
    SwingPower out;
    out.left_locked = lock_left;
    out.finite = std::isfinite(angular);
    if (!out.finite) return out;
    if (lock_left) {
        out.left = 0;
        out.right = -angular;
    } else {
        out.left = angular;
        out.right = 0;
    }
    return out;
}

// Inches traveled along a LemLib heading (0 deg faces +Y, clockwise positive).
inline double traveled_along_heading(double x, double y, double start_x, double start_y, double start_theta_deg) {
    const double dx = x - start_x;
    const double dy = y - start_y;
    const double rad = start_theta_deg * M_PI / 180.0;
    return dx * std::sin(rad) + dy * std::cos(rad);
}

inline double clamp_value(double value, double lo, double hi) {
    if (value > hi) return hi;
    if (value < lo) return lo;
    return value;
}

// max_change == 0 means no limit, matching lemlib::slew.
inline double slew_towards(double target, double current, double max_change) {
    if (max_change == 0.0) return target;
    const double change = target - current;
    if (change > max_change) return current + max_change;
    if (change < -max_change) return current - max_change;
    return target;
}

} // namespace ez_compat
