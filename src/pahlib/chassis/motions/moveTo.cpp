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
    int crossed_target_counter = 0;

    // Calculate target pose
    Pose target(x, y);
    const float initialDistance = target.distance(getPose());
    
    this->setMotionProfile(initialDistance, params.maxSpeed, params.maxAcceleration);

    const float settleDistance = std::max(2.0f, initialDistance * 0.1f);
    float adaptiveMaxSpeed = params.maxSpeed;
    
    const float minAngularDistance = 2.0f;
    const float angularErrorThreshold = 30.0f;
    float lastDistTarget = initialDistance;
    
    // Main control loop
    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose(true, true);

        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);
        const float targetHeading = pose.angle(target);

        if (distTarget < settleDistance && !settling) {
            settling = true;
            adaptiveMaxSpeed = std::max(30.0f, std::min(60.0f, std::fabs(prevLateralOut)));
        }

        // FIXED BUG #2: The original math was flawed. This version correctly projects the robot's
        // position onto the path's direction vector to check if it has passed the target.
        const float distance_past_target = (pose.x - target.x) * std::cos(targetHeading) + (pose.y - target.y) * std::sin(targetHeading);
        const bool has_crossed = distance_past_target > params.earlyExitRange;
        
        if (has_crossed) crossed_target_counter++;
        else crossed_target_counter = 0;

        if (crossed_target_counter > HYSTERESIS_CYCLES && params.minSpeed > 0 && settling) break;

        // FIXED BUG #6: Calculate errors with proper angle normalization
        const float adjustedRobotTheta = params.forwards ? pose.theta : sanitizeAngle(pose.theta + M_PI, true);
        const float angularError = angleError(adjustedRobotTheta, targetHeading);
        const float cosError = std::cos(angleError(pose.theta, targetHeading));
        const float lateralError = distTarget * cosError;

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        if (settling && lateralSmallExit.getExit() && std::fabs(radToDeg(angularError)) < 3.0f) break;

        float feedforwardVel = this->getTargetVelocity(timer.getTimePassed() / 1000.0f);
        float feedforwardAccel = this->getTargetAcceleration(timer.getTimePassed() / 1000.0f);
        
        feedforwardVel *= std::max(0.3f, std::fabs(cosError));
        
        float lateralOut = lateralPID.update(lateralError, feedforwardVel);
        lateralOut += feedforwardAccel * 0.08f;
        
        float angularOut = angularPID.update(radToDeg(angularError));

        if (settling) {
            float distanceScale = std::clamp((distTarget - 1.0f) / minAngularDistance, 0.0f, 1.0f);
            float errorScale = std::fabs(radToDeg(angularError)) < angularErrorThreshold ? 1.0f : 0.0f;
            bool movingAway = distTarget > (lastDistTarget + 0.1f) && distTarget > 1.5f;
            float awayScale = movingAway ? 0.0f : 1.0f;
            float angularScale = distanceScale * errorScale * awayScale;
            angularOut *= angularScale;
        }
        
        lastDistTarget = distTarget;

        angularOut = std::clamp(angularOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);
        lateralOut = std::clamp(lateralOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        if (!settling) {
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
            angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);
        }

        // FIXED BUG #3: The original logic was inverted. This correctly clamps the output to 0,
        // enforcing the desired direction of travel.
        if (params.forwards && !settling) lateralOut = std::max(lateralOut, 0.0f);
        else if (!params.forwards && !settling) lateralOut = std::min(lateralOut, 0.0f);

        if (params.minSpeed > 0 && !settling) {
            if (params.forwards && lateralOut > 0 && lateralOut < params.minSpeed) lateralOut = params.minSpeed;
            else if (!params.forwards && lateralOut < 0 && -lateralOut < params.minSpeed) lateralOut = -params.minSpeed;
        }

        prevLateralOut = lateralOut;
        prevAngularOut = angularOut;

        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;

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

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    
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
    if (!params.forwards) target.theta = sanitizeAngle(target.theta + M_PI, true);

    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool lateralSettled = false;
    bool angularSettled = false;
    bool settling = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    int crossed_target_counter = 0;

    const float initialDistance = target.distance(getPose());
    this->setMotionProfile(initialDistance, params.maxSpeed, params.maxAcceleration);

    const float settleDistance = std::max(params.settleDist, initialDistance * 0.08f);
    float adaptiveMaxSpeed = params.maxSpeed;
    float adaptiveLead = params.lead;

    while (!timer.isDone() && this->motionRunning) {
        const Pose pose = getPose(true, true);

        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);

        if (distTarget < settleDistance && !settling) {
            settling = true;
            adaptiveMaxSpeed = std::max(40.0f, std::min(80.0f, std::fabs(prevLateralOut)));
            adaptiveLead = std::min(params.lead, 0.3f);
        }

        lateralSettled = lateralLargeExit.getExit() && lateralSmallExit.getExit();
        angularSettled = angularLargeExit.getExit() && angularSmallExit.getExit();

        const float effectiveLead = settling ? adaptiveLead * 0.5f : adaptiveLead;
        Pose carrot = target - Pose(cos(target.theta), std::sin(target.theta)) * effectiveLead * distTarget;
        if (settling) carrot = target;

        // FIXED BUG #2: Correct early exit condition for motion chaining
        const float robot_dist_past = (pose.x - target.x) * std::cos(target.theta) + (pose.y - target.y) * std::sin(target.theta);
        const bool robot_has_crossed = robot_dist_past > params.earlyExitRange;

        const float carrot_dist_past = (carrot.x - target.x) * std::cos(target.theta) + (carrot.y - target.y) * std::sin(target.theta);
        const bool carrot_has_crossed = carrot_dist_past > params.earlyExitRange;
        
        if (robot_has_crossed != carrot_has_crossed) crossed_target_counter++;
        else crossed_target_counter = 0;

        if (crossed_target_counter > HYSTERESIS_CYCLES && params.minSpeed > 0 && settling && lateralSettled) break;

        // FIXED BUG #6: Calculate errors with proper angle normalization
        const float adjustedRobotTheta = params.forwards ? pose.theta : sanitizeAngle(pose.theta + M_PI, true);
        const float targetAngle = settling ? target.theta : pose.angle(carrot);
        const float angularError = angleError(adjustedRobotTheta, targetAngle);
        
        float lateralError = pose.distance(carrot);
        if (settling) lateralError *= std::cos(angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(radToDeg(angularError));
        angularLargeExit.update(radToDeg(angularError));

        if (settling && lateralSettled && angularSettled) break;

        float feedforwardVel = this->getTargetVelocity(timer.getTimePassed() / 1000.0f);
        float feedforwardAccel = this->getTargetAcceleration(timer.getTimePassed() / 1000.0f);
        
        float lateralOut = lateralPID.update(lateralError, feedforwardVel);
        lateralOut += feedforwardAccel * 0.08f;
        
        float angularOut = angularPID.update(radToDeg(angularError));

        if (settling) {
            const float angularScale = std::max(0.2f, std::min(1.0f, distTarget / settleDistance));
            angularOut *= angularScale;
        }

        angularOut = std::clamp(angularOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);
        lateralOut = std::clamp(lateralOut, -adaptiveMaxSpeed, adaptiveMaxSpeed);

        if (!settling) {
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
            angularOut = slew(angularOut, prevAngularOut, angularSettings.slew * 0.8f);
        }

        if (!settling) {
            const float radius = 1.0f / std::max(0.01f, std::fabs(getCurvature(pose, carrot)));
            const float maxSlipSpeed = sqrt(params.horizontalDrift * radius * 9.8f);
            lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        }

        const float totalPower = std::fabs(angularOut) + std::fabs(lateralOut);
        if (totalPower > adaptiveMaxSpeed) {
            const float angularWeight = settling ? 0.7f : 0.5f;
            const float excessPower = totalPower - adaptiveMaxSpeed;
            const float lateralReduction = excessPower * (1.0f - angularWeight);
            lateralOut = lateralOut > 0 ? std::max(0.0f, lateralOut - lateralReduction) : 
                                        std::min(0.0f, lateralOut + lateralReduction);
        }

        // Direction constraints (already correct in this function)
        if (params.forwards && !settling) lateralOut = std::max(lateralOut, 0.0f);
        else if (!params.forwards && !settling) lateralOut = std::min(lateralOut, 0.0f);

        if (params.minSpeed > 0 && !settling) {
            if (params.forwards && lateralOut > 0 && lateralOut < params.minSpeed) lateralOut = params.minSpeed;
            else if (!params.forwards && lateralOut < 0 && -lateralOut < params.minSpeed) lateralOut = -params.minSpeed;
        }

        prevLateralOut = lateralOut;
        prevAngularOut = angularOut;

        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
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

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    
    this->lateralPID = originalLateralPID;
    this->angularPID = originalAngularPID;
    this->endMotion();
}