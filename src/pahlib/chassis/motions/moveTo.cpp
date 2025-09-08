#include <optional>
#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "robot_config.hpp"

void pahlib::Chassis::moveTo(float x, float y, int timeout, MoveToPointParams params, std::optional<PIDGains> lateralGains, std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalLateralPID = this->lateralPID;
    pahlib::PID originalAngularPID = this->angularPID;

    // Apply custom PID settings if they are provided
    if (lateralGains) this->lateralPID = {lateralGains->kP, lateralGains->kI, lateralGains->kD, lateralGains->kF};
    if (angularGains) this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};

    params.earlyExitRange = std::fabs(params.earlyExitRange);
    this->requestMotionStart();
    
    // Check if all motions were cancelled
    if (!this->motionRunning) {
        this->lateralPID = originalLateralPID;
        this->angularPID = originalAngularPID;
        return;
    }
    
    // If the function is async, run it in a new task
    if (async) {
        pros::Task task([=, this]() {moveTo(x, y, timeout, params, lateralGains, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
    }

    // Reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    // Initialize variables
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool settling = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    int crossed_target_counter = 0; // HYSTERESIS: counter for early exit

    // Calculate target pose
    Pose target(x, y);
    const float initialDistance = target.distance(getPose());
    
    // Set motion profile
    this->setMotionProfile(initialDistance, params.maxSpeed, params.maxAcceleration);

    // Adaptive settling parameters
    const float settleDistance = std::max(2.0f, initialDistance * 0.1f);
    float adaptiveMaxSpeed = params.maxSpeed;
    
    // Angular control parameters
    const float minAngularDistance = 2.0f; // Distance which to start reducing angular correction, increase if gets jittery
    const float angularErrorThreshold = 30.0f; // Large angular errors suggest we crossed the target
    float lastDistTarget = initialDistance; // Track if we're moving away from target
    
    // Main control loop
    while (!timer.isDone() && this->motionRunning) {
        // Update position
        const Pose pose = getPose(true, true);

        // Update distance traveled
        const float deltaDistance = pose.distance(lastPose);
        distTraveled += deltaDistance;
        lastPose = pose;

        // Calculate distance to target
        const float distTarget = pose.distance(target);
        
        // Determine target heading (point towards target)
        const float targetHeading = pose.angle(target);

        // Adaptive speed control based on distance
        if (distTarget < settleDistance && !settling) {
            settling = true;
            adaptiveMaxSpeed = std::max(30.0f, std::min(60.0f, std::fabs(prevLateralOut)));
        }

        // HYSTERESIS: Early exit condition for motion chaining
        const bool has_crossed = (pose.y - target.y) * -sin(targetHeading) > 
                                 (pose.x - target.x) * cos(targetHeading) + params.earlyExitRange;
        
        if (has_crossed) crossed_target_counter++;
        else crossed_target_counter = 0; // Reset counter if not crossed

        // HYSTERESIS_CYCLES defined in robot_config.hpp as 3
        if (crossed_target_counter > HYSTERESIS_CYCLES && params.minSpeed > 0 && settling) break;

        // Calculate errors
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = angleError(adjustedRobotTheta, targetHeading);
        const float cosError = cos(angleError(pose.theta, targetHeading));
        const float lateralError = distTarget * cosError;

        // Update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // Check if we should exit
        if (settling && lateralSmallExit.getExit() && std::fabs(radToDeg(angularError)) < 3.0f) break;

        // Get PID outputs with feedforward
        float feedforwardVel = this->getTargetVelocity(timer.getTimePassed() / 1000.0f);
        float feedforwardAccel = this->getTargetAcceleration(timer.getTimePassed() / 1000.0f);
        
        // Scale feedforward based on alignment
        feedforwardVel *= std::max(0.3f, std::fabs(cosError));
        
        float lateralOut = lateralPID.update(lateralError, feedforwardVel);
        lateralOut += feedforwardAccel * 0.08f; // Reduced feedforward gain
        
        // Calculate angular output with improved settling behavior
        float angularOut = angularPID.update(radToDeg(angularError));

        // Adaptive angular control during settling
        if (settling) {
            // Calculate angular scaling factors
            
            // 1. Scales from 1 down to 0 as we get closer. Clamped to prevent amplification bug.
            float distanceScale = std::clamp((distTarget - 1.0f) / minAngularDistance, 0.0f, 1.0f);
            
            // 2. Kills angular output if error is huge (we likely crossed the target).
            float errorScale = std::fabs(radToDeg(angularError)) < angularErrorThreshold ? 1.0f : 0.0f;
            
            // 3. More sensitive check for moving away from target
            bool movingAway = distTarget > (lastDistTarget + 0.1f) && distTarget > 1.5f;
            float awayScale = movingAway ? 0.0f : 1.0f;
            
            // Combine the most effective scaling factors (removed alignmentScale to avoid fighting PID)
            float angularScale = distanceScale * errorScale * awayScale;
            angularOut *= angularScale;
        }
        
        // Update last distance for movement direction tracking
        lastDistTarget = distTarget;

        // Apply speed constraints
        angularOut = std::clamp(angularOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);
        lateralOut = std::clamp(lateralOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Apply slew rate limiting
        if (!settling) {
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
            angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);
        }

        // Direction constraints
        if (params.forwards && !settling) lateralOut = std::max(lateralOut, -10.0f);
        else if (!params.forwards && !settling) lateralOut = std::min(lateralOut, 10.0f);
        

        // Minimum speed constraints (apply after slew)
        if (params.minSpeed > 0 && !settling) {
            if (params.forwards && lateralOut > 0 && lateralOut < params.minSpeed) lateralOut = params.minSpeed;
            else if (!params.forwards && lateralOut < 0 && -lateralOut < params.minSpeed) lateralOut = -params.minSpeed;
        }

        // Store previous outputs
        prevLateralOut = lateralOut;
        prevAngularOut = angularOut;

        // Calculate motor powers
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;

        // Normalize to respect max speed
        const float maxPower = std::max(std::fabs(leftPower), std::fabs(rightPower));
        if (maxPower > adaptiveMaxSpeed) {
            const float ratio = adaptiveMaxSpeed / maxPower;
            leftPower *= ratio;
            rightPower *= ratio;
        }

        // Move drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        pros::delay(10);
    }

    // Stop drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    
    // Restore original PID settings
    this->lateralPID = originalLateralPID;
    this->angularPID = originalAngularPID;
    this->endMotion();
}

void pahlib::Chassis::moveTo(float x, float y, float theta, int timeout, MoveToPoseParams params, std::optional<PIDGains> lateralGains, std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings
    pahlib::PID originalLateralPID = this->lateralPID;
    pahlib::PID originalAngularPID = this->angularPID;

    // Apply custom PID settings if provided
    if (lateralGains) this->lateralPID = {lateralGains->kP, lateralGains->kI, lateralGains->kD, lateralGains->kF};
    if (angularGains) this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};

    this->requestMotionStart();
    
    if (!this->motionRunning) {
        this->lateralPID = originalLateralPID;
        this->angularPID = originalAngularPID;
        return;
    }
    
    if (async) {
        pros::Task task([=, this]() {moveTo(x, y, theta, timeout, params, lateralGains, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
    }

    // Reset controllers
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    // Calculate target pose
    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = std::fmod(target.theta + M_PI, 2.0f * M_PI);

    // Use global horizontal drift if not specified
    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    // Initialize variables
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool lateralSettled = false;
    bool angularSettled = false;
    bool settling = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    int crossed_target_counter = 0; // HYSTERESIS: counter for early exit

    const float initialDistance = target.distance(getPose());
    this->setMotionProfile(initialDistance, params.maxSpeed, params.maxAcceleration);

    // Adaptive parameters
    const float settleDistance = std::max(params.settleDist, initialDistance * 0.08f);
    float adaptiveMaxSpeed = params.maxSpeed;
    float adaptiveLead = params.lead;

    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose(true, true);

        // Update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);

        // Transition to settling phase
        if (distTarget < settleDistance && !settling) {
            settling = true;
            adaptiveMaxSpeed = std::max(40.0f, std::min(80.0f, std::fabs(prevLateralOut)));
            adaptiveLead = std::min(params.lead, 0.3f); // Reduce lead for better accuracy
        }

        // Check settlement status
        lateralSettled = lateralLargeExit.getExit() && lateralSmallExit.getExit();
        angularSettled = angularLargeExit.getExit() && angularSmallExit.getExit();

        // Calculate carrot point with adaptive lead
        const float effectiveLead = settling ? adaptiveLead * 0.5f : adaptiveLead;
        Pose carrot = target - Pose(cos(target.theta), sin(target.theta)) * effectiveLead * distTarget;
        if (settling) carrot = target;

        // HYSTERESIS: Motion chaining logic
        const bool robot_has_crossed = (pose.y - target.y) * -sin(target.theta) > 
                                       (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool carrot_has_crossed = (carrot.y - target.y) * -sin(target.theta) > 
                                        (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
        
        if (robot_has_crossed != carrot_has_crossed) crossed_target_counter++;
        else crossed_target_counter = 0;

        // HYSTERESIS_CYCLES defined in robot_config.hpp as 3
        if (crossed_target_counter > HYSTERESIS_CYCLES && params.minSpeed > 0 && settling && lateralSettled) break;

        // Calculate errors
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float targetAngle = settling ? target.theta : pose.angle(carrot);
        const float angularError = angleError(adjustedRobotTheta, targetAngle);
        
        float lateralError = pose.distance(carrot);
        if (settling) lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));

        // Update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(radToDeg(angularError));
        angularLargeExit.update(radToDeg(angularError));

        // Check for completion
        if (settling && lateralSettled && angularSettled) break;

        // Calculate PID outputs with feedforward
        float feedforwardVel = this->getTargetVelocity(timer.getTimePassed() / 1000.0f);
        float feedforwardAccel = this->getTargetAcceleration(timer.getTimePassed() / 1000.0f);
        
        float lateralOut = lateralPID.update(lateralError, feedforwardVel);
        lateralOut += feedforwardAccel * 0.08f;
        
        float angularOut = angularPID.update(radToDeg(angularError));

        // Adaptive angular scaling in settle phase
        if (settling) {
            const float angularScale = std::max(0.2f, std::min(1.0f, distTarget / settleDistance));
            angularOut *= angularScale;
        }

        // Apply speed constraints
        angularOut = std::clamp(angularOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);
        lateralOut = std::clamp(lateralOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Slew rate limiting
        if (!settling) {
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
            angularOut = slew(angularOut, prevAngularOut, angularSettings.slew * 0.8f); // Slightly reduce angular slew
        }

        // Slip prevention
        if (!settling) {
            const float radius = 1.0f / std::max(0.01f, std::fabs(getCurvature(pose, carrot)));
            const float maxSlipSpeed = sqrt(params.horizontalDrift * radius * 9.8f);
            lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        }

        // Prioritize angular over lateral when both are large
        const float totalPower = std::fabs(angularOut) + std::fabs(lateralOut);
        if (totalPower > adaptiveMaxSpeed) {
            const float angularWeight = settling ? 0.7f : 0.5f; // Prioritize angular when settling
            const float excessPower = totalPower - adaptiveMaxSpeed;
            const float lateralReduction = excessPower * (1.0f - angularWeight);
            lateralOut = lateralOut > 0 ? std::max(0.0f, lateralOut - lateralReduction) : 
                                        std::min(0.0f, lateralOut + lateralReduction);
        }

        // Direction constraints
        if (params.forwards && !settling) lateralOut = std::max(lateralOut, 0.0f);
        else if (!params.forwards && !settling) lateralOut = std::min(lateralOut, 0.0f);

        // Minimum speed constraints (apply after slew)
        if (params.minSpeed > 0 && !settling) {
            if (params.forwards && lateralOut > 0 && lateralOut < params.minSpeed) lateralOut = params.minSpeed;
            else if (!params.forwards && lateralOut < 0 && -lateralOut < params.minSpeed) lateralOut = -params.minSpeed;
        }

        prevLateralOut = lateralOut;
        prevAngularOut = angularOut;

        // Calculate and normalize motor powers
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float maxPower = std::max(std::fabs(leftPower), std::fabs(rightPower));
        
        if (maxPower > adaptiveMaxSpeed) {
            const float ratio = adaptiveMaxSpeed / maxPower;
            leftPower *= ratio;
            rightPower *= ratio;
        }

        // Move drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        pros::delay(10);
    }

    // Stop and cleanup
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    
    this->lateralPID = originalLateralPID;
    this->angularPID = originalAngularPID;
    this->endMotion();
}