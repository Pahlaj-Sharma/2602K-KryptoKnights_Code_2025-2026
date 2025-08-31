#include "main.h"
#include "pahlib/util.hpp"
#include "pahlib/chassis/chassis.hpp"
#include "pahlib/chassis/odom.hpp"
#include "pahlib/chassis/trackingWheel.hpp"
#include "robot_config.hpp"

pahlib::OdomSensors::OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                                 TrackingWheel* horizontal2, pros::Imu* imu)
    : vertical1(vertical1),
      vertical2(vertical2),
      horizontal1(horizontal1),
      horizontal2(horizontal2),
      imu(imu) {}

pahlib::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth,
                               float wheelDiameter, float rpm, float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {}

pahlib::Chassis::Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                         OdomSensors sensors, DriveCurve* throttleCurve, DriveCurve* steerCurve)
    : drivetrain(drivetrain),
      lateralSettings(linearSettings),
      angularSettings(angularSettings),
      sensors(sensors),
      throttleCurve(throttleCurve),
      steerCurve(steerCurve),
      lateralPID(linearSettings.kP, linearSettings.kI, linearSettings.kD, linearSettings.kF, linearSettings.windupRange, true),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.kF, angularSettings.windupRange, true),
      lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
      lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout) {}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(pahlib::OdomSensors& sensors) {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        sensors.imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating());
        // exit if imu has been calibrated
        if (!isnanf(sensors.imu->get_heading()) && !std::isinf(sensors.imu->get_heading())) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        sensors.imu = nullptr;
    }
}

void pahlib::Chassis::calibrate(bool calibrateImu) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    if (sensors.vertical1 == nullptr)
        sensors.vertical1 = new pahlib::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                      -(drivetrain.trackWidth / 2), drivetrain.rpm);
    if (sensors.vertical2 == nullptr)
        sensors.vertical2 = new pahlib::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                      drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();
    setSensors(sensors, drivetrain);
    init();
    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void pahlib::Chassis::setPose(float x, float y, float theta, bool radians) {
    pahlib::setPose(pahlib::Pose(x, y, theta), radians);
}

void pahlib::Chassis::setPose(Pose pose, bool radians) { pahlib::setPose(pose, radians); }

pahlib::Pose pahlib::Chassis::getPose(bool radians, bool standardPos) {
    Pose pose = pahlib::getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = radToDeg(pose.theta);
    return pose;
}

void pahlib::Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void pahlib::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void pahlib::Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void pahlib::Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void pahlib::Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

void pahlib::Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool pahlib::Chassis::isInMotion() const { return this->motionRunning; }

void pahlib::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    pahlib::setPose(pahlib::Pose(0, 0, theta), false);
}

void pahlib::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.leftMotors->set_brake_mode_all(mode);
    drivetrain.rightMotors->set_brake_mode_all(mode);
}

void pahlib::Chassis::moveLinear(float inches, int timeout, float lead, float maxspeed, float minspeed) {
    // Get the robot's current position and orientation
    const pahlib::Pose currentPose = this->getPose(true);

    // Calculate the target position based on the current pose and distance
    const float targetX = currentPose.x + (inches * std::cos(currentPose.theta));
    const float targetY = currentPose.y + (inches * std::sin(currentPose.theta));

    // Set the chassis to move to the calculated target position
    this->moveToPose(targetX, targetY, pahlib::radToDeg(currentPose.theta), timeout, {
        .lead = lead,
        .maxSpeed = maxspeed,
        .minSpeed = minspeed
    });
}

void pahlib::Chassis::setPID(PIDPreset premade) {
    PIDConstants lateral_pid;
    PIDConstants angular_pid;

    switch (premade) {
        case PIDPreset::normal:
            lateral_pid = LATERAL_PID;
            angular_pid = ANGULAR_PID;
            break;
        case PIDPreset::fast:
            lateral_pid = F_LATERAL_PID;
            angular_pid = F_ANGULAR_PID;
            break;
        case PIDPreset::precise:
            lateral_pid = P_LATERAL_PID;
            angular_pid = P_ANGULAR_PID;
            break;
        default:
            lateral_pid = LATERAL_PID;
            angular_pid = ANGULAR_PID;
            break;
    }

    this->lateralPID = {lateral_pid.kP, lateral_pid.kI, lateral_pid.kD, lateral_pid.kF};
    this->angularPID = {angular_pid.kP, angular_pid.kI, angular_pid.kD, angular_pid.kF};
}

void pahlib::Chassis::setPID(
    float lateral_kP, float lateral_kI, float lateral_kD, float lateral_kF,
    float angular_kP, float angular_kI, float angular_kD, float angular_kF
) {
    // Set the lateral PID constants
    this->lateralPID.kP = lateral_kP;
    this->lateralPID.kI = lateral_kI;
    this->lateralPID.kD = lateral_kD;
    this->lateralPID.kF = lateral_kF;

    // Set the angular PID constants
    this->angularPID.kP = angular_kP;
    this->angularPID.kI = angular_kI;
    this->angularPID.kD = angular_kD;
    this->angularPID.kF = angular_kF;
}

// FeedForward Motion Control goes here

// Motion Profile Implementation (corrected namespace)
void pahlib::MotionProfile::setMotionProfile(float target_distance, float max_voltage, float max_acceleration) {
    // Handle direction and ensure positive calculations
    m_direction = (target_distance >= 0) ? 1 : -1;
    m_target_distance = std::fabs(target_distance);
    
    // Input validation
    if (m_target_distance < MIN_EPSILON) {
        // Zero distance - create a stationary profile
        m_max_velocity = 0;
        m_max_acceleration = 0;
        m_time_accel = 0;
        m_time_cruise = 0;
        m_time_total = 0;
        m_is_triangular = true;
        return;
    }
    
    // Convert voltage to velocity and validate inputs
    m_max_velocity = std::fabs(max_voltage) * VOLTS_TO_VELOCITY;
    m_max_acceleration = std::fabs(max_acceleration);
    
    if (m_max_velocity < MIN_EPSILON || m_max_acceleration < MIN_EPSILON) {
        // Invalid parameters - create stationary profile
        m_max_velocity = 0;
        m_max_acceleration = 0;
        m_time_accel = 0;
        m_time_cruise = 0;
        m_time_total = 0;
        m_is_triangular = true;
        return;
    }
    
    // Calculate time and distance to accelerate to max velocity
    m_time_accel = m_max_velocity / m_max_acceleration;
    float accel_distance = 0.5f * m_max_acceleration * m_time_accel * m_time_accel;
    
    // Determine if profile is triangular or trapezoidal
    if (m_target_distance < (2.0f * accel_distance)) {
        // Triangular profile - can't reach max velocity
        m_is_triangular = true;
        m_max_velocity = std::sqrt(m_max_acceleration * m_target_distance);
        m_time_accel = m_max_velocity / m_max_acceleration;
        m_time_cruise = 0;
    } else {
        // Trapezoidal profile - reaches max velocity
        m_is_triangular = false;
        m_time_cruise = (m_target_distance - (2.0f * accel_distance)) / m_max_velocity;
    }
    
    // Calculate total time
    m_time_total = (2.0f * m_time_accel) + m_time_cruise;
}

float pahlib::MotionProfile::getTargetVelocity(float elapsed_time) const {
    // Clamp time to valid range
    elapsed_time = std::max(0.0f, std::min(elapsed_time, m_time_total));
    
    // Handle edge cases
    if (m_time_total < MIN_EPSILON) {
        return 0.0f;
    }
    
    float target_velocity = 0.0f;
    
    // Determine which phase we're in
    if (elapsed_time <= m_time_accel) {
        // Acceleration phase
        target_velocity = (elapsed_time / m_time_accel) * m_max_velocity;
    } 
    else if (elapsed_time <= (m_time_accel + m_time_cruise)) {
        // Constant velocity (cruise) phase
        target_velocity = m_max_velocity;
    } 
    else {
        // Deceleration phase
        float time_into_decel = elapsed_time - (m_time_accel + m_time_cruise);
        float decel_progress = time_into_decel / m_time_accel;
        target_velocity = m_max_velocity * (1.0f - decel_progress);
    }
    
    // Apply direction
    return target_velocity * m_direction;
}

float pahlib::MotionProfile::getTargetAcceleration(float elapsed_time) const {
    // Clamp time to valid range
    elapsed_time = std::max(0.0f, std::min(elapsed_time, m_time_total));
    
    if (m_time_total < MIN_EPSILON) {
        return 0.0f;
    }
    
    float target_acceleration = 0.0f;
    
    if (elapsed_time < m_time_accel) {
        // Acceleration phase
        target_acceleration = m_max_acceleration;
    }
    else if (elapsed_time < (m_time_accel + m_time_cruise)) {
        // Cruise phase
        target_acceleration = 0.0f;
    }
    else if (elapsed_time < m_time_total) {
        // Deceleration phase
        target_acceleration = -m_max_acceleration;
    }
    
    // Apply direction
    return target_acceleration * m_direction;
}

float pahlib::MotionProfile::getTargetPosition(float elapsed_time) const {
    // Clamp time to valid range
    elapsed_time = std::max(0.0f, std::min(elapsed_time, m_time_total));
    
    if (m_time_total < MIN_EPSILON) {
        return 0.0f;
    }
    
    float target_position = 0.0f;
    
    if (elapsed_time <= m_time_accel) {
        // Acceleration phase: s = 0.5 * a * t^2
        target_position = 0.5f * m_max_acceleration * elapsed_time * elapsed_time;
    }
    else if (elapsed_time <= (m_time_accel + m_time_cruise)) {
        // Cruise phase: s = s_accel + v_max * t_cruise
        float accel_distance = 0.5f * m_max_acceleration * m_time_accel * m_time_accel;
        float cruise_time = elapsed_time - m_time_accel;
        target_position = accel_distance + (m_max_velocity * cruise_time);
    }
    else {
        // Deceleration phase
        float accel_distance = 0.5f * m_max_acceleration * m_time_accel * m_time_accel;
        float cruise_distance = m_max_velocity * m_time_cruise;
        float time_into_decel = elapsed_time - (m_time_accel + m_time_cruise);
        
        // s = v_max * t - 0.5 * a * t^2
        float decel_distance = (m_max_velocity * time_into_decel) - 
                             (0.5f * m_max_acceleration * time_into_decel * time_into_decel);
        
        target_position = accel_distance + cruise_distance + decel_distance;
    }
    
    // Apply direction
    return target_position * m_direction;
}

// Chassis wrapper functions (corrected)
void pahlib::Chassis::setMotionProfile(float target_distance, float max_voltage, float max_acceleration) {
    m_motion_profile.setMotionProfile(target_distance, max_voltage, max_acceleration);
}

float pahlib::Chassis::getTargetVelocity(float elapsed_time) {
    return m_motion_profile.getTargetVelocity(elapsed_time);
}

float pahlib::Chassis::getTargetAcceleration(float elapsed_time) {
    return m_motion_profile.getTargetAcceleration(elapsed_time);
}

float pahlib::Chassis::getTargetPosition(float elapsed_time) {
    return m_motion_profile.getTargetPosition(elapsed_time);
}