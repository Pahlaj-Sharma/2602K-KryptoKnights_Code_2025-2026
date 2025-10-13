#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "robot_config.hpp"
#include "units/Angle.hpp"

void pahlib::Chassis::turnTo(float theta, int timeout, TurnToHeadingParams params, 
                             std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalAngularPID = this->angularPID;

    // Determine if we should use gain scheduling
    const bool useGainScheduling = !angularGains && params.gainScheduling;
    AngularSchedule angularSchedule;

    // theta parameter is in compass degrees - convert to radians
    const Angle targetHeading = Angle(degToRad(theta) * rad);
    const Angle currentHeading = Angle(getPose(true).theta * rad); // getPose(true) returns compass radians
    const Angle initialError = units::abs(units::constrainAngle180(targetHeading - currentHeading));

    // Apply custom PID settings if provided
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
        pros::Task task([=, this]() {turnTo(theta, timeout, params, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
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

    // Calculate settle threshold
    const Angle settleThreshold = units::max(Angle(3_cDeg), initialError * 0.15);
    float adaptiveMaxSpeed = params.maxSpeed;

    // Main control loop
    while (!timer.isDone() && this->motionRunning) {
        const Angle currentHeading = Angle(getPose(true).theta * rad); // Compass radians
        
        // Update distance traveled (in compass degrees)
        distTraveled = to_cDeg(units::abs(units::constrainAngle180(currentHeading - startHeading)));

        // Calculate error based on direction and settling state
        Angle angleError = Angle(0_cDeg);
        if (settling) {
            // During settling, always take shortest path
            angleError = units::constrainAngle180(targetHeading - currentHeading);
        } else {
            // Before settling, respect direction parameter
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

        // Detect when we should start settling (crossed target or close enough)
        if (prevAngleError != std::nullopt) {
            if (!settling && (units::abs(angleError) < settleThreshold || 
                             units::sgn(angleError) != units::sgn(*prevAngleError))) {
                settling = true;
                adaptiveMaxSpeed = std::fmax(25.0f, std::min(60.0f, std::abs(prevMotorPower)));
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

        // Update exit conditions (use compass degrees)
        const float angleErrorDeg = to_cDeg(angleError);
        angularLargeExit.update(angleErrorDeg);
        angularSmallExit.update(angleErrorDeg);

        // Check for completion
        if (settling && angularSmallExit.getExit() && units::abs(angleError) < Angle(1_cDeg)) break;

        // Early exit for motion chaining
        if (params.minSpeed > 0 && settling && units::abs(angleError) < Angle(params.earlyExitRange * deg)) break;

        // Calculate PID output (in compass degrees)
        float motorPower = angularPID.update(angleErrorDeg);

        // Apply speed constraints
        motorPower = std::clamp(motorPower, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Apply slew rate limiting
        if (units::abs(angleError) > Angle(15_cDeg) && !settling) 
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        else if (settling)
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew * 0.7f);

        // Apply minimum speed constraints AFTER slew rate limiting
        if (params.minSpeed > 0 && !settling) {
            if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
            else if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        }

        prevMotorPower = motorPower;

        // Move drivetrain
        drivetrain.leftMotors->move(motorPower);
        drivetrain.rightMotors->move(-motorPower);

        pros::delay(10);
    }

    // Stop and cleanup
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->angularPID = originalAngularPID;
    this->endMotion();
}

void pahlib::Chassis::turnTo(float x, float y, int timeout, TurnToPointParams params, 
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
        pros::Task task([=, this]() {turnTo(x, y, timeout, params, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
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
    const Angle settleThreshold = units::max(Angle(4_cDeg), initialError * 0.12);
    float adaptiveMaxSpeed = params.maxSpeed;

    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose(true); // Compass radians
        
        // Adjust for forward/backward
        Angle currentHeading = Angle(pose.theta * rad);
        if (!params.forwards) currentHeading = currentHeading + Angle(180_cDeg);

        distTraveled = to_cDeg(units::abs(units::constrainAngle180(currentHeading - startHeading)));

        // Calculate target angle to point (pose.angle returns standard, convert to compass)
        const Angle targetHeading = from_cRad(Number(pose.angle({x, y})));

        // Calculate error based on direction and settling state
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
                adaptiveMaxSpeed = std::fmax(30.0f, std::min(65.0f, std::abs(prevMotorPower)));
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

        // Apply slew rate limiting
        if (units::abs(angleError) > Angle(15_cDeg) && !settling) 
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        else if (settling)
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew * 0.75f);

        // Apply minimum speed constraints AFTER slew rate limiting
        if (params.minSpeed > 0 && !settling) {
            if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
            else if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        }

        prevMotorPower = motorPower;

        // Move drivetrain
        drivetrain.leftMotors->move(motorPower);
        drivetrain.rightMotors->move(-motorPower);

        pros::delay(10);
    }

    // Stop and cleanup
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->angularPID = originalAngularPID;
    this->endMotion();
}