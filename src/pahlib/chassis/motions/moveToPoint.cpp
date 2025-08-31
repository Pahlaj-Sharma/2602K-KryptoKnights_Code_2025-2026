#include <cmath>
#include <optional>
#include "pahlib/chassis/chassis.hpp"
#include "pahlib/timer.hpp"
#include "pahlib/util.hpp"
#include "pros/misc.hpp"

void pahlib::Chassis::moveToPoint(float x, float y, int timeout, MoveToPointParams params, std::optional<PIDGains> lateralGains, std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::ControllerSettings originalLateral = this->lateralSettings;
    pahlib::ControllerSettings originalAngular = this->angularSettings;

    // Apply custom PID settings if they are provided
    if (lateralGains) {
        this->lateralPID = {lateralGains->kP, lateralGains->kI, lateralGains->kD, lateralGains->kF};
    }
    if (angularGains) {
        this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};
    }

    params.earlyExitRange = std::fabs(params.earlyExitRange);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) {
        this->lateralPID = {originalLateral.kP, originalLateral.kI, originalLateral.kD, originalLateral.kF};
        this->angularPID = {originalAngular.kP, originalAngular.kI, originalAngular.kD, originalAngular.kF};
        return;
    }
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, lateralGains, angularGains, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;

    // calculate target pose in standard form
    Pose target(x, y);
    target.theta = lastPose.angle(target);

    this->setMotionProfile(target.distance(getPose()), params.maxSpeed,
                           params.maxAcceleration);

    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        // update position
        const Pose pose = getPose(true, true);

        // update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // calculate distance to the target point
        const float distTarget = pose.distance(target);

        // check if the robot is close enough to the target to start settling
        if (distTarget < 4 && close == false) {
            close = true;
            params.maxSpeed = fmax(std::fabs(prevLateralOut), 60);
        }

        // motion chaining
        const bool side =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
        // exit if close
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = angleError(adjustedRobotTheta, pose.angle(target));
        float lateralError = pose.distance(target) * cos(angleError(pose.theta, pose.angle(target)));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // get output from PIDs
        float feedforward_velocity = this->getTargetVelocity(timer.getTimePassed() / 1000.0f);
        float lateralOut = lateralPID.update(lateralError, feedforward_velocity);
        float feedforward_accel = this->getTargetAcceleration(timer.getTimePassed() / 1000.0);
        lateralOut += feedforward_accel * 0.1f;  // Adjust the 0.1f gain as needed!!
        float angularOut = angularPID.update(radToDeg(angularError));
        if (close) angularOut = 0;

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
        // constrain lateral output by max accel
        // but not for decelerating, since that would interfere with settling
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // prevent moving in the wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < std::fabs(params.minSpeed) && lateralOut > 0) lateralOut = std::fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < std::fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -std::fabs(params.minSpeed);

        // update previous output
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->lateralPID = {originalLateral.kP, originalLateral.kI, originalLateral.kD, originalLateral.kF};
    this->angularPID = {originalAngular.kP, originalAngular.kI, originalAngular.kD, originalAngular.kF};
    this->endMotion();
}