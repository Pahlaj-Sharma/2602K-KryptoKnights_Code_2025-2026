#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "robot_config.hpp"
#include "units/Angle.hpp"

void pahlib::Chassis::moveTo(float x, float y, int timeout, MoveToPointParams params, std::optional<PIDGains> lateralGains, std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings for restoration
    pahlib::PID originalLateralPID = this->lateralPID;
    pahlib::PID originalAngularPID = this->angularPID;

    // Initialize gain scheduling if no custom gains provided
    const bool useGainScheduling = !lateralGains && !angularGains && params.gainScheduling;
    LateralSchedule lateralSchedule;
    AngularSchedule angularSchedule;

    Pose target(x, y);
    
    const float initialDistance = target.distance(getPose(true));
    
    // Apply custom PID gains or calculate initial scheduled gains
    if (lateralGains || angularGains) {
        if (lateralGains) this->lateralPID = {lateralGains->kP, lateralGains->kI, lateralGains->kD, lateralGains->kF};
        if (angularGains) this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};
    } else if (useGainScheduling) {
        Pose currentPose = getPose(true); // Compass radians
        
        // pose.angle() returns standard position, need to convert to compass
        const Angle targetHeading = from_cRad(Number(currentPose.angle(target))); // Compass
        const Angle robotHeading = Angle(currentPose.theta * rad); // Already compass, just wrap in Angle
        const Angle adjustedRobotHeading = params.forwards ? robotHeading : robotHeading + Angle(180_cDeg);

        const Angle initialAngularError = units::constrainAngle180(targetHeading - adjustedRobotHeading);

        PIDGains scheduledLateralGains = interpolateGains(
            initialDistance, lateralSchedule.distances, lateralSchedule.gains);
        PIDGains scheduledAngularGains = interpolateGains(
            to_cDeg(units::abs(initialAngularError)), angularSchedule.angles, angularSchedule.gains);
        
        this->lateralPID = {scheduledLateralGains.kP, scheduledLateralGains.kI, 
                            scheduledLateralGains.kD, scheduledLateralGains.kF};
        this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                            scheduledAngularGains.kD, scheduledAngularGains.kF};
    }
    
    params.earlyExitRange = std::fabs(params.earlyExitRange);
    
    this->requestMotionStart();
    
    // Exit if motion was cancelled
    if (!this->motionRunning) {
        this->lateralPID = originalLateralPID;
        this->angularPID = originalAngularPID;
        return;
    }
    
    // Run asynchronously in a new task if requested
    if (async) {
        pros::Task task([=, this]() {moveTo(x, y, timeout, params, lateralGains, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
    }

    // Reset all controllers and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    // Initialize control loop variables
    Pose lastPose = getPose(true);
    distTraveled = 0;
    Timer timer(timeout);
    bool settling = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    int crossed_target_counter = 0;
    
    // Configure motion profile
    this->setMotionProfile(initialDistance, params.maxSpeed, params.maxAcceleration);

    const float settleDistance = std::max(2.0f, initialDistance * 0.1f);
    float adaptiveMaxSpeed = params.maxSpeed;
    
    const float minAngularDistance = 2.0f;
    const Angle angularErrorThreshold = Angle(30_cDeg);
    float lastDistTarget = initialDistance;
    
    // Main control loop
    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose(true); // Compass radians

        // Update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);
        
        // pose.angle() returns standard position, convert to compass
        const Angle targetHeading = from_cRad(Number(pose.angle(target))); // Compass

        // Enter settling phase when close to target
        if (distTarget < settleDistance && !settling) {
            settling = true;
            adaptiveMaxSpeed = std::max(30.0f, std::min(60.0f, std::fabs(prevLateralOut)));
        }

        // Check if robot has crossed target point (for early exit)
        // Use standard position for dot product calculations
        const float distance_past_target = (pose.x - target.x) * std::cos(pose.angle(target)) + 
                                           (pose.y - target.y) * std::sin(pose.angle(target));
        const bool has_crossed = distance_past_target > params.earlyExitRange;
        
        if (has_crossed) crossed_target_counter++;
        else crossed_target_counter = 0;

        if (crossed_target_counter > 3 && params.minSpeed > 0 && settling) break;

        // Calculate errors in compass orientation
        const Angle robotHeading = Angle(pose.theta * rad); // pose.theta is already compass radians
        const Angle adjustedRobotHeading = params.forwards ? robotHeading : robotHeading + Angle(180_cDeg);
        
        // Calculate angular error (shortest path, constrained to [-180, 180] degrees)
        const Angle angularError = units::constrainAngle180(targetHeading - adjustedRobotHeading);

        // Calculate lateral error projection using compass angles
        const Angle headingError = units::constrainAngle180(targetHeading - robotHeading);
        const float cosError = units::cos(headingError);
        const float lateralError = distTarget * cosError;

        // Apply dynamic gain scheduling during settling
        if (useGainScheduling && settling) {
            PIDGains scheduledLateralGains = interpolateGains(
                std::fabs(lateralError), lateralSchedule.distances, lateralSchedule.gains);
            PIDGains scheduledAngularGains = interpolateGains(
                to_cDeg(units::abs(angularError)), angularSchedule.angles, angularSchedule.gains);
            
            this->lateralPID = {scheduledLateralGains.kP, scheduledLateralGains.kI, 
                                scheduledLateralGains.kD, scheduledLateralGains.kF};
            
            this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                                 scheduledAngularGains.kD, scheduledAngularGains.kF};
        }

        // Update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // Exit if settled
        if (settling && lateralSmallExit.getExit() && units::abs(angularError) < Angle(3_cDeg)) break;

        // Calculate feedforward components
        float feedforwardVel = this->getTargetVelocity(timer.getTimePassed() / 1000.0f);
        float feedforwardAccel = this->getTargetAcceleration(timer.getTimePassed() / 1000.0f);
        
        // When cosError is negative, we're moving away from target, so reduce feedforward
        if (cosError < 0) {
            feedforwardVel *= 0.3f; // Significantly reduce when moving away
        } else {
            // When aligned, use full feedforward. When perpendicular, use 50%
            feedforwardVel *= std::max(0.5f, cosError);
        }
        
        // Calculate PID outputs (PID uses compass degrees)
        float lateralOut = lateralPID.update(lateralError, feedforwardVel);
        lateralOut += feedforwardAccel * 0.08f;
        
        // CRITICAL: Negate angular output because compass is CW-positive but motors expect CCW-positive (standard)
        float angularOut = angularPID.update(to_cDeg(angularError));

        // Apply settling or slew rate limiting
        if (settling) {
            // Reduce angular correction when close to target, aligned, and not moving away
            const float distanceScale = std::clamp((distTarget - 1.0f) / minAngularDistance, 0.0f, 1.0f);
            const float errorScale = units::abs(angularError) < angularErrorThreshold ? 1.0f : 0.0f;
            const bool movingAway = distTarget > (lastDistTarget + 0.1f) && distTarget > 1.5f;
            const float awayScale = movingAway ? 0.0f : 1.0f;
            const float angularScale = distanceScale * errorScale * awayScale;
            angularOut *= angularScale;
        } else {
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
            angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);
        }
        
        lastDistTarget = distTarget;

        // Clamp outputs to max speed
        angularOut = std::clamp(angularOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);
        lateralOut = std::clamp(lateralOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Enforce forward/backward direction when not settling
        if (params.forwards && !settling) lateralOut = std::max(lateralOut, 0.0f);
        else if (!params.forwards && !settling) lateralOut = std::min(lateralOut, 0.0f);

        // Apply minimum speed when not settling
        if (params.minSpeed > 0 && !settling) {
            if (params.forwards && lateralOut > 0 && lateralOut < params.minSpeed) lateralOut = params.minSpeed;
            else if (!params.forwards && lateralOut < 0 && -lateralOut < params.minSpeed) lateralOut = -params.minSpeed;
        }

        prevLateralOut = lateralOut;
        prevAngularOut = angularOut;

        // Calculate wheel powers
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;

        // Normalize if exceeding max speed
        const float maxPower = std::max(std::fabs(leftPower), std::fabs(rightPower));
        if (maxPower > adaptiveMaxSpeed) {
            const float ratio = adaptiveMaxSpeed / maxPower;
            leftPower *= ratio;
            rightPower *= ratio;
        }

        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        pros::delay(10);
    }

    // Stop motors and cleanup
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    
    this->lateralPID = originalLateralPID;
    this->angularPID = originalAngularPID;
    this->endMotion();
}

void pahlib::Chassis::moveTo(float x, float y, float theta, int timeout, MoveToPoseParams params, std::optional<PIDGains> lateralGains, std::optional<PIDGains> angularGains, bool async) {
    // Store original PID settings for restoration
    pahlib::PID originalLateralPID = this->lateralPID;
    pahlib::PID originalAngularPID = this->angularPID;

    // Initialize gain scheduling if no custom gains provided
    const bool useGainScheduling = !lateralGains && !angularGains && params.gainScheduling;
    LateralSchedule lateralSchedule;
    AngularSchedule angularSchedule;

    // theta parameter is in compass degrees (0° = North)
    // Convert directly to compass radians (no conversion needed, just change units)
    Pose target(x, y, degToRad(theta)); // Compass radians (internal storage)

    const float initialDistance = target.distance(getPose(true));
    
    // Apply custom PID gains or calculate initial scheduled gains
    if (lateralGains || angularGains) {
        if (lateralGains) this->lateralPID = {lateralGains->kP, lateralGains->kI, lateralGains->kD, lateralGains->kF};
        if (angularGains) this->angularPID = {angularGains->kP, angularGains->kI, angularGains->kD, angularGains->kF};
    } else if (useGainScheduling) {
        // Convert compass to standard for calculations
        Angle targetHeading = Angle(target.theta * rad); // Target is in compass radians
        const Angle initialTargetHeading = params.forwards ? targetHeading : targetHeading + Angle(180_cDeg);
        
        Pose currentPose = getPose(true); // Compass radians
        const Angle robotHeading = Angle(currentPose.theta * rad); // Already compass
        const Angle adjustedRobotHeading = params.forwards ? robotHeading : robotHeading + Angle(180_cDeg);

        const Angle initialAngularError = units::constrainAngle180(initialTargetHeading - adjustedRobotHeading);
        
        PIDGains scheduledLateralGains = interpolateGains(
            initialDistance, lateralSchedule.distances, lateralSchedule.gains);
        PIDGains scheduledAngularGains = interpolateGains(
            to_cDeg(units::abs(initialAngularError)), angularSchedule.angles, angularSchedule.gains);
        
        this->lateralPID = {scheduledLateralGains.kP, scheduledLateralGains.kI, 
                           scheduledLateralGains.kD, scheduledLateralGains.kF};
        this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                           scheduledAngularGains.kD, scheduledAngularGains.kF};
    }

    params.earlyExitRange = std::fabs(params.earlyExitRange);

    this->requestMotionStart();
    
    // Exit if motion was cancelled
    if (!this->motionRunning) {
        this->lateralPID = originalLateralPID;
        this->angularPID = originalAngularPID;
        return;
    }
    
    // Run asynchronously in a new task if requested
    if (async) {
        pros::Task task([=, this]() {moveTo(x, y, theta, timeout, params, lateralGains, angularGains, false);});
        pros::delay(10);
        this->endMotion();
        return;
    }

    // Reset all controllers and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    // Adjust target heading if backing up (target.theta is already compass radians)
    if (!params.forwards) target.theta = sanitizeAngle(target.theta + M_PI, true);

    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    // Initialize control loop variables
    Pose lastPose = getPose(true);
    distTraveled = 0;
    Timer timer(timeout);
    bool lateralSettled = false;
    bool angularSettled = false;
    bool settling = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    int crossed_target_counter = 0;

    this->setMotionProfile(initialDistance, params.maxSpeed, params.maxAcceleration);

    const float settleDistance = std::max(params.settleDist, initialDistance * 0.08f);
    float adaptiveMaxSpeed = params.maxSpeed;
    float adaptiveLead = params.lead;

    // Main control loop
    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose(true); // Compass radians

        // Update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);

        // Enter settling phase when close to target
        if (distTarget < settleDistance && !settling) {
            settling = true;
            adaptiveMaxSpeed = std::max(40.0f, std::min(80.0f, std::fabs(prevLateralOut)));
            adaptiveLead = std::min(params.lead, 0.3f);
        }

        // Check if both lateral and angular are settled
        lateralSettled = lateralLargeExit.getExit() && lateralSmallExit.getExit();
        angularSettled = angularLargeExit.getExit() && angularSmallExit.getExit();

        // Calculate carrot point (lookahead point along target heading)
        const float effectiveLead = settling ? adaptiveLead * 0.5f : adaptiveLead;
        // target.theta is compass radians, convert to standard position for trig functions
        // Standard = 90° - Compass
        const float targetThetaStandard = M_PI_2 - target.theta;
        Pose carrot = target - Pose(std::cos(targetThetaStandard), std::sin(targetThetaStandard)) * effectiveLead * distTarget;
        if (settling) carrot = target;

        // Check if robot and carrot have crossed target (for early exit)
        // Use standard position angles for dot product
        const float robot_dist_past = (pose.x - target.x) * std::cos(targetThetaStandard) + 
                                     (pose.y - target.y) * std::sin(targetThetaStandard);
        const bool robot_has_crossed = robot_dist_past > params.earlyExitRange;

        // For carrot, use the same target heading direction for consistency
        const float carrot_dist_past = (carrot.x - target.x) * std::cos(targetThetaStandard) + 
                                      (carrot.y - target.y) * std::sin(targetThetaStandard);
        const bool carrot_has_crossed = carrot_dist_past > params.earlyExitRange;
        
        if (robot_has_crossed != carrot_has_crossed) crossed_target_counter++;
        else crossed_target_counter = 0;

        if (crossed_target_counter > 3 && params.minSpeed > 0 && settling && lateralSettled) break;

        // Calculate errors in compass orientation
        const Angle robotHeading = Angle(pose.theta * rad); // pose.theta is already compass radians
        const Angle adjustedRobotHeading = params.forwards ? robotHeading : robotHeading + Angle(180_cDeg);
        const Angle carrotHeading = from_cRad(Number(pose.angle(carrot))); // pose.angle() returns standard, convert to compass
        const Angle finalTargetHeading = Angle(target.theta * rad); // target.theta is already compass radians
        const Angle targetAngle = settling ? finalTargetHeading : carrotHeading;

        // Calculate angular error (shortest path)
        const Angle angularError = units::constrainAngle180(targetAngle - adjustedRobotHeading);
        
        // Calculate lateral error
        float lateralError = pose.distance(carrot);
        const Angle carrotHeadingError = units::constrainAngle180(carrotHeading - robotHeading);
        
        if (settling) {
            lateralError *= units::cos(carrotHeadingError);
        } else {
            lateralError *= units::sgn(units::cos(carrotHeadingError));
        }

        // Apply dynamic gain scheduling during settling
        if (useGainScheduling && settling) {
            PIDGains scheduledLateralGains = interpolateGains(
                std::fabs(lateralError), lateralSchedule.distances, lateralSchedule.gains);
            PIDGains scheduledAngularGains = interpolateGains(
                to_cDeg(units::abs(angularError)), angularSchedule.angles, angularSchedule.gains);
            
            this->lateralPID = {scheduledLateralGains.kP, scheduledLateralGains.kI, 
                                scheduledLateralGains.kD, scheduledLateralGains.kF};
            
            this->angularPID = {scheduledAngularGains.kP, scheduledAngularGains.kI, 
                                 scheduledAngularGains.kD, scheduledAngularGains.kF};
        }

        // Update exit conditions (use compass degrees)
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(to_cDeg(angularError));
        angularLargeExit.update(to_cDeg(angularError));

        // Exit if both settled
        if (settling && lateralSettled && angularSettled) break;

        // Calculate feedforward components
        const float feedforwardVel = this->getTargetVelocity(timer.getTimePassed() / 1000.0f);
        const float feedforwardAccel = this->getTargetAcceleration(timer.getTimePassed() / 1000.0f);
        
        // Calculate PID outputs (PID uses compass degrees)
        float lateralOut = lateralPID.update(lateralError, feedforwardVel);
        lateralOut += feedforwardAccel * 0.08f;
        
        // CRITICAL: Negate angular output because compass is CW-positive but motors expect CCW-positive (standard)
        float angularOut = angularPID.update(to_cDeg(angularError));

        // Apply settling or slew rate limiting with slip prevention
        if (settling) {
            // Scale down angular correction as distance decreases
            const float angularScale = std::max(0.2f, std::min(1.0f, distTarget / settleDistance));
            angularOut *= angularScale;
        } else {
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
            angularOut = slew(angularOut, prevAngularOut, angularSettings.slew * 0.8f);
            
            // Limit speed based on curve radius to prevent slipping
            const float radius = 1.0f / std::max(0.01f, std::fabs(getCurvature(pose, carrot)));
            const float maxSlipSpeed = sqrt(params.horizontalDrift * radius * 9.8f);
            lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        }

        // Clamp outputs to max speed
        angularOut = std::clamp(angularOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);
        lateralOut = std::clamp(lateralOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        // Prioritize angular correction if total power exceeds limit
        const float totalPower = std::fabs(angularOut) + std::fabs(lateralOut);
        if (totalPower > adaptiveMaxSpeed) {
            const float angularWeight = settling ? 0.7f : 0.5f;
            const float excessPower = totalPower - adaptiveMaxSpeed;
            const float lateralReduction = excessPower * (1.0f - angularWeight);
            lateralOut = lateralOut > 0 ? std::max(0.0f, lateralOut - lateralReduction) : 
                                        std::min(0.0f, lateralOut + lateralReduction);
        }

        // Enforce forward/backward direction when not settling
        if (params.forwards && !settling) lateralOut = std::max(lateralOut, 0.0f);
        else if (!params.forwards && !settling) lateralOut = std::min(lateralOut, 0.0f);

        // Apply minimum speed when not settling
        if (params.minSpeed > 0 && !settling) {
            if (params.forwards && lateralOut > 0 && lateralOut < params.minSpeed) lateralOut = params.minSpeed;
            else if (!params.forwards && lateralOut < 0 && -lateralOut < params.minSpeed) lateralOut = -params.minSpeed;
        }

        prevLateralOut = lateralOut;
        prevAngularOut = angularOut;

        // Calculate wheel powers
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float maxPower = std::max(std::fabs(leftPower), std::fabs(rightPower));
        
        // Normalize if exceeding max speed
        if (maxPower > adaptiveMaxSpeed) {
            const float ratio = adaptiveMaxSpeed / maxPower;
            leftPower *= ratio;
            rightPower *= ratio;
        }

        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        pros::delay(10);
    }

    // Stop motors and cleanup
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    
    this->lateralPID = originalLateralPID;
    this->angularPID = originalAngularPID;
    this->endMotion();
}