#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "robot_config.hpp"
#include "units/Angle.hpp"

void pahlib::Chassis::swingTo(float theta, DriveSide lockedSide, int timeout, SwingToHeadingParams params,
                              std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalAngularPID = this->angularPID;

    // Determine if we should use gain scheduling
    const bool useGainScheduling = !angularGains && params.gainScheduling;
    AngularSchedule angularSchedule;

    // theta parameter is in compass degrees - convert to radians
    const Angle targetHeading = Angle(degToRad(theta) * rad);
    const Angle currentHeading = Angle(getPose(true).theta * rad); // Compass radians
    const Angle initialError = units::abs(units::constrainAngle180(targetHeading - currentHeading));

    if (angularGains) {
        this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};
    } else if (useGainScheduling) {
        PIDGains scheduledAngularGains = interpolateGains(
            to_cDeg(initialError), angularSchedule.angles, angularSchedule.gains);
        
        this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                           scheduledAngularGains.kD, scheduledAngularGains.kF};
    }

    params.minSpeed = std::abs(params.minSpeed);
    params.earlyExitRange = std::abs(params.earlyExitRange);
    
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
    const Angle startHeading = Angle(getPose(true).theta * rad);
    float prevMotorPower = 0;
    bool settling = false;
    std::optional<Angle> prevAngleError = std::nullopt;
    
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // Calculate settle threshold (slightly higher for swing)
    const Angle settleThreshold = units::max(Angle(4_cDeg), initialError * 0.18);
    float adaptiveMaxSpeed = params.maxSpeed;

    while (!timer.isDone() && this->motionRunning) {
        const Angle currentHeading = Angle(getPose(true).theta * rad);
        distTraveled = to_cDeg(units::abs(units::constrainAngle180(currentHeading - startHeading)));

        // Calculate error
        Angle angleError = Angle(0_cDeg);
        if (settling) {
            angleError = units::constrainAngle180(targetHeading - currentHeading);
        } else {
            Angle rawError = targetHeading - currentHeading;
            switch (params.direction) {
                case AngularDirection::CW_CLOCKWISE:
                    angleError = rawError < Angle(0_cDeg) ? rawError + Angle(360_cDeg) : rawError;
                    break;
                case AngularDirection::CCW_COUNTERCLOCKWISE:
                    angleError = rawError > Angle(0_cDeg) ? rawError - Angle(360_cDeg) : rawError;
                    break;
                default: // AUTO
                    angleError = units::constrainAngle180(rawError);
                    break;
            }
        }

        // Detect settling condition
        if (prevAngleError != std::nullopt) {
            if (!settling && (units::abs(angleError) < settleThreshold || 
                             units::sgn(angleError) != units::sgn(*prevAngleError))) {
                settling = true;
                // More conservative speed reduction for swing motions
                adaptiveMaxSpeed = std::fmax(20.0f, std::min(50.0f, std::abs(prevMotorPower)));
            }
        }
        prevAngleError = angleError;

        // Apply dynamic gain scheduling ONLY during settling phase
        if (useGainScheduling && settling) {
            PIDGains scheduledAngularGains = interpolateGains(
                to_cDeg(units::abs(angleError)), angularSchedule.angles, angularSchedule.gains);
            
            this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                                 scheduledAngularGains.kD, scheduledAngularGains.kF};
        }

        // Update exit conditions
        const float angleErrorDeg = to_cDeg(angleError);
        angularLargeExit.update(angleErrorDeg);
        angularSmallExit.update(angleErrorDeg);

        // Check for completion
        if (settling && angularSmallExit.getExit() && units::abs(angleError) < Angle(1.2_cDeg)) break;

        // Early exit for motion chaining
        if (params.minSpeed > 0 && settling && units::abs(angleError) < Angle(params.earlyExitRange * deg)) break;

        // Calculate PID output
        float motorPower = angularPID.update(angleErrorDeg);

        // Apply speed constraints
        motorPower = std::clamp(motorPower, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // More conservative slew rate for swing motions to prevent wheel slip
        if (units::abs(angleError) > Angle(12_cDeg) && !settling)
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
    const bool useGainScheduling = !angularGains && params.gainScheduling;
    AngularSchedule angularSchedule;

    Pose currentPose = getPose(true); // Compass radians

    if (angularGains) {
        this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};
    } else if (useGainScheduling) {
        // pose.angle() returns standard position, convert to compass
        const Angle initialTargetHeading = from_cRad(Number(currentPose.angle({x, y})));
        const Angle currentHeading = Angle(currentPose.theta * rad); // Already compass
        const Angle adjustedCurrentHeading = params.forwards ? currentHeading : currentHeading + Angle(180_cDeg);
        const Angle initialError = units::abs(units::constrainAngle180(initialTargetHeading - adjustedCurrentHeading));
        
        PIDGains scheduledAngularGains = interpolateGains(
            to_cDeg(initialError), angularSchedule.angles, angularSchedule.gains);
        
        this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                           scheduledAngularGains.kD, scheduledAngularGains.kF};
    }

    params.minSpeed = std::abs(params.minSpeed);
    params.earlyExitRange = std::abs(params.earlyExitRange);
    
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
    const Angle startHeading = Angle(getPose(true).theta * rad);
    float prevMotorPower = 0;
    bool settling = false;
    std::optional<Angle> prevAngleError = std::nullopt;
    
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // Calculate initial error for settle threshold
    const Angle initialTargetHeading = from_cRad(Number(currentPose.angle({x, y})));
    const Angle initialCurrentHeading = Angle(currentPose.theta * rad);
    const Angle adjustedInitialHeading = params.forwards ? initialCurrentHeading : initialCurrentHeading + Angle(180_cDeg);
    const Angle initialError = units::abs(units::constrainAngle180(initialTargetHeading - adjustedInitialHeading));
    const Angle settleThreshold = units::max(Angle(5_cDeg), initialError * 0.15);
    float adaptiveMaxSpeed = params.maxSpeed;

    while (!timer.isDone() && this->motionRunning) {
        Pose pose = getPose(true); // Compass radians
        
        // Adjust heading for backward movement
        Angle currentHeading = Angle(pose.theta * rad);
        if (!params.forwards) currentHeading = currentHeading + Angle(180_cDeg);

        distTraveled = to_cDeg(units::abs(units::constrainAngle180(currentHeading - startHeading)));

        // Calculate target angle to point (pose.angle returns standard, convert to compass)
        const Angle targetHeading = from_cRad(Number(pose.angle({x, y})));

        // Calculate error
        Angle angleError = Angle(0_cDeg);
        if (settling) {
            angleError = units::constrainAngle180(targetHeading - currentHeading);
        } else {
            Angle rawError = targetHeading - currentHeading;
            switch (params.direction) {
                case AngularDirection::CW_CLOCKWISE:
                    angleError = rawError < Angle(0_cDeg) ? rawError + Angle(360_cDeg) : rawError;
                    break;
                case AngularDirection::CCW_COUNTERCLOCKWISE:
                    angleError = rawError > Angle(0_cDeg) ? rawError - Angle(360_cDeg) : rawError;
                    break;
                default: // AUTO
                    angleError = units::constrainAngle180(rawError);
                    break;
            }
        }

        // Detect settling condition
        if (prevAngleError != std::nullopt) {
            if (!settling && (units::abs(angleError) < settleThreshold || 
                             units::sgn(angleError) != units::sgn(*prevAngleError))) {
                settling = true;
                adaptiveMaxSpeed = std::fmax(25.0f, std::min(55.0f, std::abs(prevMotorPower)));
            }
        }
        prevAngleError = angleError;

        // Apply dynamic gain scheduling ONLY during settling phase
        if (useGainScheduling && settling) {
            PIDGains scheduledAngularGains = interpolateGains(
                to_cDeg(units::abs(angleError)), angularSchedule.angles, angularSchedule.gains);
            
            this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                                 scheduledAngularGains.kD, scheduledAngularGains.kF};
        }

        // Update exit conditions
        const float angleErrorDeg = to_cDeg(angleError);
        angularLargeExit.update(angleErrorDeg);
        angularSmallExit.update(angleErrorDeg);

        // Check for completion
        if (settling && angularSmallExit.getExit() && units::abs(angleError) < Angle(1.5_cDeg)) break;

        // Early exit for motion chaining
        if (params.minSpeed > 0 && settling && units::abs(angleError) < Angle(params.earlyExitRange * deg)) break;

        // Calculate PID output
        float motorPower = angularPID.update(angleErrorDeg);

        // Apply speed constraints
        motorPower = std::clamp(motorPower, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Conservative slew rate for swing to point
        if (units::abs(angleError) > Angle(10_cDeg) && !settling)
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