#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"

void pahlib::Chassis::turnTo(float theta, int timeout, TurnToHeadingParams params, 
                             std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalAngularPID = this->angularPID;

    // Apply custom PID settings if provided
    if (angularGains) this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};

    params.minSpeed = std::abs(params.minSpeed);
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
    const float startTheta = getPose().theta;
    float prevMotorPower = 0;
    bool settling = false;
    std::optional<float> prevDeltaTheta = std::nullopt;
    
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // Calculate initial error to determine if we need to settle quickly
    const float initialError = std::abs(angleError(theta, getPose().theta, false));
    const float settleThreshold = std::max(3.0f, initialError * 0.15f);
    float adaptiveMaxSpeed = params.maxSpeed;

    // Main control loop
    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose();
        
        // Update distance traveled
        distTraveled = std::abs(angleError(pose.theta, startTheta, false));

        // Calculate error with direction consideration
        float deltaTheta;
        if (settling) deltaTheta = angleError(theta, pose.theta, false);
        else deltaTheta = angleError(theta, pose.theta, false, params.direction);

        // Detect when we should start settling (crossed target or close enough)
        if (prevDeltaTheta != std::nullopt) {
            if (!settling && (std::abs(deltaTheta) < settleThreshold || 
                             sgn(deltaTheta) != sgn(*prevDeltaTheta))) {
                settling = true;
                adaptiveMaxSpeed = std::max(25.0f, std::min(60.0f, std::abs(prevMotorPower)));
            }
        }
        prevDeltaTheta = deltaTheta;

        // Update exit conditions
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // Check for completion
        if (settling && angularSmallExit.getExit() && std::abs(deltaTheta) < 1.0f) break;

        // Early exit for motion chaining
        if (params.minSpeed > 0 && settling && std::abs(deltaTheta) < params.earlyExitRange) break;

        // Calculate PID output
        float motorPower = angularPID.update(deltaTheta);

        // Apply speed constraints
        motorPower = std::clamp(motorPower, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Apply slew rate limiting more aggressively when not settling
        if (std::abs(deltaTheta) > 15.0f && !settling) 
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        else if (settling) // Gentler slew when settling
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

    if (angularGains) this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};

    params.minSpeed = std::abs(params.minSpeed);
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
    const float startTheta = getPose().theta;
    float prevMotorPower = 0;
    bool settling = false;
    std::optional<float> prevDeltaTheta = std::nullopt;
    
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // Calculate initial target to determine settle threshold
    Pose currentPose = getPose();
    const float deltaX = x - currentPose.x;
    const float deltaY = y - currentPose.y;
    const float initialTargetTheta = std::fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)) + 360.0f, 360.0f);
    const float initialError = std::abs(angleError(initialTargetTheta, currentPose.theta, false));
    const float settleThreshold = std::max(4.0f, initialError * 0.12f);
    float adaptiveMaxSpeed = params.maxSpeed;

    while (!timer.isDone() && this->motionRunning) {
        Pose pose = getPose();
        
        // Adjust pose theta based on forward/backward movement
        if (!params.forwards) pose.theta = std::fmod(pose.theta + 180.0f, 360.0f);

        distTraveled = std::abs(angleError(pose.theta, startTheta, false));

        // Calculate target angle to point
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
                adaptiveMaxSpeed = std::max(30.0f, std::min(65.0f, std::abs(prevMotorPower)));
            }
        }
        prevDeltaTheta = deltaTheta;

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

        // Apply slew rate limiting
        if (std::abs(deltaTheta) > 15.0f && !settling) 
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