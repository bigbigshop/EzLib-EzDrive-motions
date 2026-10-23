#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"
#include "ez_compat/motion_math.hpp"

void lemlib::Chassis::moveToPoint(float x, float y, int timeout, MoveToPointParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    if (!this->motionRunning) {
        this->endMotion();
        return;
    }
    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    resetMotionPIDs();
    lateralLargeExit.reset();
    lateralSmallExit.reset();

    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    bool lateralDone = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;

    Pose target(x, y);
    target.theta = lastPose.angle(target);

    while (!timer.isDone() && (!lateralDone || !close) && this->motionRunning) {
        const Pose pose = getPose(true, true);

        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);

        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        const bool side = (pose.y - target.y) * -sin(target.theta) <=
                          (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        const ez_compat::PointErrors pointErr = ez_compat::move_to_point_errors(
            {pose.x, pose.y, pose.theta}, {target.x, target.y, 0}, params.forwards);
        float lateralError = static_cast<float>(pointErr.lateral_in);

        float lateralOut = 0;
        float angularOut = 0;
        if (pointErr.finite) {
            lateralOut = commandLateral(lateralError);
            angularOut = commandAngular(static_cast<float>(pointErr.angular_deg));
            lateralDone = lateralMotionSettled();
        }
        if (close) angularOut = 0;

        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSlewStep());

        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSlewStep());

        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        infoSink()->debug("Angular Out: {}, Lateral Out: {}", angularOut, lateralOut);

        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }
        if (!std::isfinite(leftPower) || !std::isfinite(rightPower)) {
            leftPower = 0;
            rightPower = 0;
        }

        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        pros::delay(10);
    }

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}
