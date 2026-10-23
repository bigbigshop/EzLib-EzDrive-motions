#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"
#include "ez_compat/motion_math.hpp"

void lemlib::Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) {
        this->endMotion();
        return;
    }
    if (async) {
        pros::Task task([&]() { moveToPose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    resetMotionPIDs();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI);

    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    bool lateralDone = false;
    bool angularDone = false;
    bool prevSameSide = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    const int compState = pros::competition::get_status();

    while (!timer.isDone() && (!(lateralDone && angularDone) || !close) && this->motionRunning) {
        const Pose pose = getPose(true, true);

        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);

        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        const ez_compat::PoseErrors poseErr = ez_compat::move_to_pose_errors(
            {pose.x, pose.y, pose.theta}, {target.x, target.y, target.theta}, params.forwards, params.lead, close);
        Pose carrot(static_cast<float>(poseErr.carrot_x), static_cast<float>(poseErr.carrot_y), target.theta);

        const bool robotSide = (pose.y - target.y) * -sin(target.theta) <=
                               (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool carrotSide = (carrot.y - target.y) * -sin(target.theta) <=
                                (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool sameSide = robotSide == carrotSide;
        if (!sameSide && prevSameSide && close && params.minSpeed != 0) break;
        prevSameSide = sameSide;

        float lateralError = static_cast<float>(poseErr.lateral_in);

        float lateralOut = 0;
        float angularOut = 0;
        if (poseErr.finite) {
            lateralOut = commandLateral(lateralError);
            angularOut = commandAngular(static_cast<float>(poseErr.angular_deg));
            lateralDone = lateralMotionSettled();
            angularDone = angularMotionSettled();
        }

        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSlewStep());

        const float radius = 1 / fabs(getCurvature(pose, carrot));
        const float maxSlipSpeed(sqrt(params.horizontalDrift * radius * 9.8));
        lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        const float overturn = fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        infoSink()->debug("lateralOut: {} angularOut: {}", lateralOut, angularOut);

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
