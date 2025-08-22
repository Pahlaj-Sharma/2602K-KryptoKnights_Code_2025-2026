#include <cmath>
#include "pahlib/chassis/chassis.hpp"
#include "pahlib/logger/logger.hpp"
#include "pahlib/timer.hpp"
#include "pahlib/util.hpp"
#include "pros/misc.hpp"

void pahlib::Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params, std::optional<PIDGains> lateralGains, std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::ControllerSettings originalLateral = this->lateralSettings;
    pahlib::ControllerSettings originalAngular = this->angularSettings;

    // Apply custom PID settings if they are provided
    if (lateralGains) {
        this->lateralPID.kP = lateralGains->kP;
        this->lateralPID.kI = lateralGains->kI;
        this->lateralPID.kD = lateralGains->kD;
        this->lateralPID.kF = lateralGains->kF;
    }
    if (angularGains) {
        this->angularPID.kP = angularGains->kP;
        this->angularPID.kI = angularGains->kI;
        this->angularPID.kD = angularGains->kD;
        this->angularPID.kF = angularGains->kF;
    }

    // take the mutex
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) {
        this->lateralPID = {originalLateral.kP, originalLateral.kI, originalLateral.kD, originalLateral.kF};
        this->angularPID = {originalAngular.kP, originalAngular.kI, originalAngular.kD, originalAngular.kF};
        return;
    }
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPose(x, y, theta, timeout, params, lateralGains, angularGains, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    // calculate target pose in standard form
    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = std::fmod(target.theta + M_PI, M_TWOPI); // backwards movement

    // use global horizontalDrift is horizontalDrift is 0
    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    bool lateralSettled = false;
    bool prevSameSide = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();

    this->setMotionProfile(target.distance(getPose()), params.maxSpeed / 2.54,
                           params.maxAcceleration);

    // main loop
    while (!timer.isDone() &&
           ((!lateralSettled || (!angularLargeExit.getExit() && !angularSmallExit.getExit())) || !close) &&
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

        // check if the lateral controller has settled
        if (lateralLargeExit.getExit() && lateralSmallExit.getExit()) lateralSettled = true;

        // calculate the carrot point
        Pose carrot = target - Pose(cos(target.theta), sin(target.theta)) * params.lead * distTarget;
        if (close) carrot = target; // settling behavior

        // calculate if the robot is on the same side as the carrot point
        const bool robotSide =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool carrotSide = (carrot.y - target.y) * -sin(target.theta) <=
                                (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool sameSide = robotSide == carrotSide;
        // exit if close
        if (!sameSide && prevSameSide && close && params.minSpeed != -1.0f) break;
        prevSameSide = sameSide;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError =
            close ? angleError(adjustedRobotTheta, target.theta) : angleError(adjustedRobotTheta, pose.angle(carrot));
        float lateralError = pose.distance(carrot);
        // only use cos when settling
        // otherwise just multiply by the sign of cos
        // maxSlipSpeed takes care of lateralOut
        if (close) lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(radToDeg(angularError));
        angularLargeExit.update(radToDeg(angularError));

        // get output from PIDs
        float lateralOut = lateralPID.update(lateralError, this->getTargetVelocity(timer.getTimePassed() / 1000.0));
        float angularOut = angularPID.update(radToDeg(angularError));

        if (distTarget < params.settleDist) {
            // The scaling factor will be between 0 and 1, getting smaller as distTarget approaches 0.
            angularOut *= std::tanh(distTarget / params.settleDist);
        }

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        // constrain lateral output by max accel
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // constrain lateral output by the max speed it can travel at without
        // slipping
        const float radius = 1 / std::fabs(getCurvature(pose, carrot));
        const float maxSlipSpeed(sqrt(params.horizontalDrift * radius * 9.8));
        lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        // prioritize angular movement over lateral movement
        const float overturn = std::fabs(angularOut) + std::fabs(lateralOut) - params.maxSpeed;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

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

        infoSink()->debug("lateralOut: {} angularOut: {}", lateralOut, angularOut);

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
