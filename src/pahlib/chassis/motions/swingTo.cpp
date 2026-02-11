#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "robot_config.hpp"
#include "units/Angle.hpp"

void pahlib::Chassis::swingTo(float theta, DriveSide lockedSide, int timeout, SwingToHeadingParams params,
                              std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalAngularPID = this->angularPID;
    drivetrain.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    drivetrain.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
params.minSpeed = fabs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { swingTo(theta, lockedSide, timeout, params, angularGains, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();
    // get original braking mode of that side of the drivetrain so we can set it back to it after this motion ends
    pros::MotorBrake brakeMode = (lockedSide == DriveSide::LEFT)
                                     ? this->drivetrain.leftMotors->get_brake_mode_all().at(0)
                                     : this->drivetrain.rightMotors->get_brake_mode_all().at(0);
    // set brake mode of the locked side to hold
    if (lockedSide == DriveSide::LEFT) this->drivetrain.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    else this->drivetrain.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();
        pose.theta = fmod(pose.theta, 360);

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));
        targetTheta = theta;

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        // move the drivetrain
        if (lockedSide == DriveSide::LEFT) {
            drivetrain.rightMotors->move(-motorPower);
            drivetrain.leftMotors->brake();
        } else {
            drivetrain.leftMotors->move(motorPower);
            drivetrain.rightMotors->brake();
        }

        // delay to save resources
        pros::delay(10);
    }

    // set the brake mode of the locked side of the drivetrain to its
    // original value
    if (lockedSide == DriveSide::LEFT) this->drivetrain.leftMotors->set_brake_mode_all(brakeMode);
    else this->drivetrain.rightMotors->set_brake_mode_all(brakeMode);
    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->angularPID = originalAngularPID;
    this->endMotion();
}

void pahlib::Chassis::swingTo(float x, float y, DriveSide lockedSide, int timeout, SwingToPointParams params,
                              std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalAngularPID = this->angularPID;
    drivetrain.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    drivetrain.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    params.minSpeed = fabs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { swingTo(x, y, lockedSide, timeout, params, angularGains, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaX, deltaY, deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();
    // get original braking mode of that side of the drivetrain so we can set it back to it after this motion ends
    pros::MotorBrake brakeMode = (lockedSide == DriveSide::LEFT)
                                     ? this->drivetrain.leftMotors->get_brake_mode_all().at(0)
                                     : this->drivetrain.rightMotors->get_brake_mode_all().at(0);
    // set brake mode of the locked side to hold
    if (lockedSide == DriveSide::LEFT) this->drivetrain.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    else this->drivetrain.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();
        pose.theta = (params.forwards) ? fmod(pose.theta, 360) : fmod(pose.theta - 180, 360);

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        // move the drivetrain
        if (lockedSide == DriveSide::LEFT) {
            drivetrain.rightMotors->move(-motorPower);
            drivetrain.leftMotors->brake();
        } else {
            drivetrain.leftMotors->move(motorPower);
            drivetrain.rightMotors->brake();
        }

        pros::delay(10);
    }

    // set the brake mode of the locked side of the drivetrain to its
    // original value
    if (lockedSide == DriveSide::LEFT) this->drivetrain.leftMotors->set_brake_mode_all(brakeMode);
    else this->drivetrain.rightMotors->set_brake_mode_all(brakeMode);
    // stop the drivetrain
    drivetrain.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    drivetrain.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->angularPID = originalAngularPID;
    this->endMotion();
}