#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"
#include "ez_compat/motion_math.hpp"

namespace {
ez_compat::TurnDir toTurnDir(lemlib::AngularDirection direction) {
    if (direction == lemlib::AngularDirection::CW_CLOCKWISE) return ez_compat::TurnDir::CW;
    if (direction == lemlib::AngularDirection::CCW_COUNTERCLOCKWISE) return ez_compat::TurnDir::CCW;
    return ez_compat::TurnDir::Auto;
}
} // namespace

void lemlib::Chassis::turnToPoint(float x, float y, int timeout, TurnToPointParams params, bool async) {
    params.minSpeed = std::abs(params.minSpeed);
    this->requestMotionStart();
    if (!this->motionRunning) {
        this->endMotion();
        return;
    }
    if (async) {
        pros::Task task([&]() { turnToPoint(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }
    float targetTheta;
    float deltaX, deltaY, deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    bool angularDone = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    resetMotionPIDs();

    while (!timer.isDone() && !angularDone && this->motionRunning) {
        Pose pose = getPose();
        pose.theta = (params.forwards) ? fmod(pose.theta, 360) : fmod(pose.theta - 180, 360);

        distTraveled = fabs(ez_compat::heading_error_deg(pose.theta, startTheta));

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        const float rawDeltaTheta = ez_compat::heading_error_deg(targetTheta, pose.theta, ez_compat::TurnDir::Auto);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        const ez_compat::TurnDir dir = settling ? ez_compat::TurnDir::Auto : toTurnDir(params.direction);
        deltaTheta = ez_compat::heading_error_deg(targetTheta, pose.theta, dir);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        motorPower = commandAngular(deltaTheta);
        angularDone = angularMotionSettled();

        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSlewStep());
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        infoSink()->debug("Turn Motor Power: {} ", motorPower);

        if (!std::isfinite(motorPower)) motorPower = 0;
        drivetrain.leftMotors->move(motorPower);
        drivetrain.rightMotors->move(-motorPower);

        pros::delay(10);
    }

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}
