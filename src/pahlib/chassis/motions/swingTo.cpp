#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "robot_config.hpp"

void pahlib::Chassis::swingTo(float theta, DriveSide lockedSide, int timeout, SwingToHeadingParams params,
                              std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalAngularPID = this->angularPID;

    // Determine if we should use gain scheduling
    bool useGainScheduling = !angularGains && params.gainScheduling;
    AngularSchedule angularSchedule;

    const float initialError = std::abs(angleError(theta, getPose().theta, false));

    if (angularGains) {
        this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};
    } else if (useGainScheduling) {
        // Calculate static PID gains for gain scheduling based on initial angular error
        
        PIDGains scheduledAngularGains = interpolateGains(
            initialError, angularSchedule.angles, angularSchedule.gains);
        
        this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                           scheduledAngularGains.kD, scheduledAngularGains.kF};
    }

    params.minSpeed = std::abs(params.minSpeed);
    this->requestMotionStart();
    
    if (!this->motionRunning) {
        this->angularPID = originalAngularPID;
        return;
    }
    
    if (async) {
        pros::Task task([=, this]() {swingTo(theta, lockedSide, timeout, params, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
    }

    // Get and store original brake mode
    pros::MotorBrake originalBrakeMode;
    if (lockedSide == DriveSide::LEFT) {
        originalBrakeMode = drivetrain.leftMotors->get_brake_mode_all().at(0);
        drivetrain.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    } else {
        originalBrakeMode = drivetrain.rightMotors->get_brake_mode_all().at(0);
        drivetrain.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    }

    // Initialize variables
    const float startTheta = getPose().theta;
    float prevMotorPower = 0;
    bool settling = false;
    std::optional<float> prevDeltaTheta = std::nullopt;
    
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // Calculate initial error and settle threshold
    const float settleThreshold = std::fmax(4.0f, initialError * 0.18f); // Slightly higher for swing
    float adaptiveMaxSpeed = params.maxSpeed;

    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose();
        distTraveled = std::abs(angleError(pose.theta, startTheta, false));

        // Calculate error
        float deltaTheta;
        if (settling) deltaTheta = angleError(theta, pose.theta, false);
        else deltaTheta = angleError(theta, pose.theta, false, params.direction);

        // Detect settling condition
        if (prevDeltaTheta != std::nullopt) {
            if (!settling && (std::abs(deltaTheta) < settleThreshold || 
                             sgn(deltaTheta) != sgn(*prevDeltaTheta))) {
                settling = true;
                // More conservative speed reduction for swing motions
                adaptiveMaxSpeed = std::fmax(20.0f, std::min(50.0f, std::abs(prevMotorPower)));
            }
        }
        prevDeltaTheta = deltaTheta;

        // Apply dynamic gain scheduling ONLY during settling phase
        if (useGainScheduling && settling) {
            PIDGains scheduledAngularGains = interpolateGains(
                std::abs(deltaTheta), angularSchedule.angles, angularSchedule.gains);
            
            this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                                 scheduledAngularGains.kD, scheduledAngularGains.kF};
        }

        // Update exit conditions
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // Check for completion
        if (settling && angularSmallExit.getExit() && std::abs(deltaTheta) < 1.2f) break;

        // Early exit for motion chaining
        if (params.minSpeed > 0 && settling && std::abs(deltaTheta) < params.earlyExitRange) break;

        // Calculate PID output
        float motorPower = angularPID.update(deltaTheta);

        // Apply speed constraints
        motorPower = std::clamp(motorPower, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // More conservative slew rate for swing motions to prevent wheel slip
        if (std::abs(deltaTheta) > 12.0f && !settling)
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew * 0.8f);
        else if (settling)
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew * 0.6f);

        // Apply minimum speed constraints AFTER slew rate limiting
        if (params.minSpeed > 0 && !settling) {
            if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
            else if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        }

        prevMotorPower = motorPower;

        // Move drivetrain (swing motion)
        if (lockedSide == DriveSide::LEFT) {
            drivetrain.rightMotors->move(-motorPower);
            drivetrain.leftMotors->brake();
        } else {
            drivetrain.leftMotors->move(motorPower);
            drivetrain.rightMotors->brake();
        }

        pros::delay(10);
    }

    // Restore original brake mode and stop
    if (lockedSide == DriveSide::LEFT) drivetrain.leftMotors->set_brake_mode_all(originalBrakeMode);
    else drivetrain.rightMotors->set_brake_mode_all(originalBrakeMode);
    
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->angularPID = originalAngularPID;
    this->endMotion();
}

void pahlib::Chassis::swingTo(float x, float y, DriveSide lockedSide, int timeout, SwingToPointParams params,
                              std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalAngularPID = this->angularPID;

    // Determine if we should use gain scheduling
    bool useGainScheduling = !angularGains && params.gainScheduling;
    AngularSchedule angularSchedule;

    Pose currentPose = getPose();

    if (angularGains) {
        this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};
    } else if (useGainScheduling) {
        // Calculate static PID gains for gain scheduling based on initial angular error
        const float deltaX = x - currentPose.x;
        const float deltaY = y - currentPose.y;
        const float initialTargetTheta = std::fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)) + 360.0f, 360.0f);
        const float initialError = std::abs(angleError(initialTargetTheta, currentPose.theta, false));
        
        PIDGains scheduledAngularGains = interpolateGains(
            initialError, angularSchedule.angles, angularSchedule.gains);
        
        this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                           scheduledAngularGains.kD, scheduledAngularGains.kF};
    }

    params.minSpeed = std::abs(params.minSpeed);
    this->requestMotionStart();
    
    if (!this->motionRunning) {
        this->angularPID = originalAngularPID;
        return;
    }
    
    if (async) {
        pros::Task task([=, this]() {swingTo(x, y, lockedSide, timeout, params, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
    }

    // Get and store original brake mode
    pros::MotorBrake originalBrakeMode;
    if (lockedSide == DriveSide::LEFT) {
        originalBrakeMode = drivetrain.leftMotors->get_brake_mode_all().at(0);
        drivetrain.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    } else {
        originalBrakeMode = drivetrain.rightMotors->get_brake_mode_all().at(0);
        drivetrain.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    }

    // Initialize variables
    const float startTheta = getPose().theta;
    float prevMotorPower = 0;
    bool settling = false;
    std::optional<float> prevDeltaTheta = std::nullopt;
    
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // Calculate initial target and settle threshold
    const float deltaX = x - currentPose.x;
    const float deltaY = y - currentPose.y;
    const float initialTargetTheta = std::fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)) + 360.0f, 360.0f);
    const float initialError = std::abs(angleError(initialTargetTheta, currentPose.theta, false));
    const float settleThreshold = std::fmax(5.0f, initialError * 0.15f);
    float adaptiveMaxSpeed = params.maxSpeed;

    while (!timer.isDone() && this->motionRunning) {
        Pose pose = getPose();
        
        // Adjust pose theta for backward movement
        if (!params.forwards) pose.theta = std::fmod(pose.theta + 180.0f, 360.0f);

        distTraveled = std::abs(angleError(pose.theta, startTheta, false));

        // Calculate target angle
        const float dx = x - pose.x;
        const float dy = y - pose.y;
        const float targetTheta = std::fmod(radToDeg(M_PI_2 - atan2(dy, dx)) + 360.0f, 360.0f);

        // Calculate error
        float deltaTheta;
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);

        // Detect settling condition
        if (prevDeltaTheta != std::nullopt) {
            if (!settling && (std::abs(deltaTheta) < settleThreshold || 
                             sgn(deltaTheta) != sgn(*prevDeltaTheta))) {
                settling = true;
                adaptiveMaxSpeed = std::fmax(25.0f, std::min(55.0f, std::abs(prevMotorPower)));
            }
        }
        prevDeltaTheta = deltaTheta;

        // Apply dynamic gain scheduling ONLY during settling phase
        if (useGainScheduling && settling) {
            PIDGains scheduledAngularGains = interpolateGains(
                std::abs(deltaTheta), angularSchedule.angles, angularSchedule.gains);
            
            this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                                 scheduledAngularGains.kD, scheduledAngularGains.kF};
        }

        // Update exit conditions
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // Check for completion
        if (settling && angularSmallExit.getExit() && std::abs(deltaTheta) < 1.5f) break;

        // Early exit for motion chaining
        if (params.minSpeed > 0 && settling && std::abs(deltaTheta) < params.earlyExitRange) break;

        // Calculate PID output
        float motorPower = angularPID.update(deltaTheta);

        // Apply speed constraints
        motorPower = std::clamp(motorPower, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Conservative slew rate for swing to point
        if (std::abs(deltaTheta) > 10.0f && !settling)
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew * 0.85f);
        else if (settling)
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew * 0.65f);

        // Apply minimum speed constraints AFTER slew rate limiting
        if (params.minSpeed > 0 && !settling) {
            if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
            else if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        }

        prevMotorPower = motorPower;

        // Move drivetrain (swing motion)
        if (lockedSide == DriveSide::LEFT) {
            drivetrain.rightMotors->move(-motorPower);
            drivetrain.leftMotors->brake();
        } else {
            drivetrain.leftMotors->move(motorPower);
            drivetrain.rightMotors->brake();
        }

        pros::delay(10);
    }

    // Restore original brake mode and stop
    if (lockedSide == DriveSide::LEFT) drivetrain.leftMotors->set_brake_mode_all(originalBrakeMode);
    else drivetrain.rightMotors->set_brake_mode_all(originalBrakeMode);
    
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->angularPID = originalAngularPID;
    this->endMotion();
}