#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP
#include "pahlib/chassis/chassis.hpp"
// --- Motor Ports ---
// Define motor port numbers for clarity and easy modification.

inline constexpr int PORT_RIGHT_FRONT  = 10; //14
inline constexpr int PORT_RIGHT_MIDDLE = 20;
inline constexpr int PORT_RIGHT_BACK   = 18;
inline constexpr int PORT_LEFT_FRONT   = -6; //fix
inline constexpr int PORT_LEFT_MIDDLE  = -19; //19
inline constexpr int PORT_LEFT_BACK    = -7;
inline constexpr int PORT_INTAKE_MOTOR = 9;
//inline constexpr int PORT_RIGHT_PTO    = -7;
inline constexpr int PORT_SCORE_MOTOR  = 1;

// --- Sensor Ports ---
inline constexpr int PORT_IMU                = 13; // Inertial Measurement Unit
inline constexpr int PORT_HORIZONTAL_ENCODER = 5; // Horizontal tracking wheel encoder
inline constexpr int PORT_VERTICAL_ENCODER   = -14;  // Vertical tracking wheel encoder
inline constexpr int PORT_AUTON_SELECTOR     = 5;  // Rotation for autonomous routine selection
//inline constexpr int PORT_PTO                = 4;  // Digital output for PTO control
inline constexpr int PORT_DISTANCE_RIGHT     = 8;
inline constexpr int PORT_DISTANCE_LEFT      = 4;
inline constexpr int PORT_DISTANCE_FRONT     = 3;
inline constexpr int PORT_DISTANCE_BACK      = 21;
inline constexpr int PORT_MATCH_LOAD         = 2;
inline constexpr int PORT_CENTER_GOAL        = 3;
inline constexpr int PORT_DOUBLE_PARK        = 4;
inline constexpr int PORT_ANTENNE            = 1;

// --- Drivetrain Constants (in inches/RPM as appropriate) ---
inline constexpr float TRACK_WIDTH      = 11.25; // Distance between the centers of the left and right wheels in inches
inline constexpr int   WHEEL_RPM        = 450;   // Max effective RPM of the drivetrain (e.g., 600 RPM blue motors with 1.33:1 external gearing)
inline constexpr float HORIZONTAL_DRIFT = 2.0f;   // Horizontal drift in inches, used for odometry calculations
inline constexpr float IMU_SCALER       = 1.00912f;   // Custom IMU scaling factor, adjust based on IMU's calibration
 
// --- Odometry Tracking Wheel Offsets ---
// Offsets from the robot's center to the tracking wheel in inches.
inline constexpr float VERTICAL_TRACKING_OFFSET   = 0.0f;
inline constexpr float HORIZONTAL_TRACKING_OFFSET   = -1.6875f; //1.5

// --- PID Controller Settings for pahlib Chassis ---
// A struct to hold all the PID constants for clarity and easy management.
struct PIDConstants {
    float kP;
    float kI;
    float kD;
    float kF;
    int antiWindup;
    int smallError;
    int smallTimeout;
    int largeError;
    int largeTimeout;
    int slew;
};

// Default Lateral PID constants
inline constexpr PIDConstants LATERAL_PID {
    9.5f, 0.0f, 9.5f, 0.0f, 3, 1, 100, 2, 500, 20
};
// Custom "Fast" Lateral PID constants
inline constexpr PIDConstants F_LATERAL_PID {
    7.0f, 0.0f, 9.0f, 0.0f, 3, 2, 100, 3, 500, 20
};
// Custom "Precise" Lateral PID constants
inline constexpr PIDConstants P_LATERAL_PID {
    7.0f, 0.0f, 9.0f, 0.0f, 3, 1, 100, 2, 500, 20
};
// Default Angular PID constants
inline constexpr PIDConstants ANGULAR_PID {
    4.0f, 0.0f, 25.0f, 0.0f, 3, 1, 100, 3, 500, 0
};
// Custom "Fast" Angular PID constants
inline constexpr PIDConstants F_ANGULAR_PID {
    2.0f, 0.0f, 16.0f, 0.0f, 3, 2, 100, 4, 500, 0
};
// Custom "Precise" Angular PID constants
inline constexpr PIDConstants P_ANGULAR_PID {
    2.0f, 0.0f, 16.0f, 0.0f, 3, 1, 100, 2, 500, 0
};

// Convert to pahlib ControllerSettings
// Normal Lateral
inline const pahlib::ControllerSettings lateral_PID = {
    LATERAL_PID.kP, LATERAL_PID.kI, LATERAL_PID.kD, LATERAL_PID.kF, LATERAL_PID.antiWindup, 
    LATERAL_PID.smallError, LATERAL_PID.smallTimeout, LATERAL_PID.largeError, 
    LATERAL_PID.largeTimeout, LATERAL_PID.slew};
// Normal Angular
inline const pahlib::ControllerSettings angular_PID = {
    ANGULAR_PID.kP, ANGULAR_PID.kI, ANGULAR_PID.kD, ANGULAR_PID.kF, ANGULAR_PID.antiWindup, 
    ANGULAR_PID.smallError, ANGULAR_PID.smallTimeout, ANGULAR_PID.largeError, 
    ANGULAR_PID.largeTimeout, ANGULAR_PID.slew};
// Fast Lateral
inline const pahlib::ControllerSettings F_lateral_PID = {
    F_LATERAL_PID.kP, F_LATERAL_PID.kI, F_LATERAL_PID.kD, F_ANGULAR_PID.kF, F_LATERAL_PID.antiWindup, 
    F_LATERAL_PID.smallError, F_LATERAL_PID.smallTimeout, F_LATERAL_PID.largeError, 
    F_LATERAL_PID.largeTimeout, F_LATERAL_PID.slew};
// Fast Angular
inline const pahlib::ControllerSettings F_angular_PID = {
    F_ANGULAR_PID.kP, F_ANGULAR_PID.kI, F_ANGULAR_PID.kD, F_ANGULAR_PID.kF, F_ANGULAR_PID.antiWindup, 
    F_ANGULAR_PID.smallError, F_ANGULAR_PID.smallTimeout, F_ANGULAR_PID.largeError, 
    F_ANGULAR_PID.largeTimeout, F_ANGULAR_PID.slew};
// Precise Lateral
inline const pahlib::ControllerSettings P_lateral_PID = {
    P_LATERAL_PID.kP, P_LATERAL_PID.kI, P_LATERAL_PID.kD, P_LATERAL_PID.kF, P_LATERAL_PID.antiWindup, 
    P_LATERAL_PID.smallError, P_LATERAL_PID.smallTimeout, P_LATERAL_PID.largeError, 
    P_LATERAL_PID.largeTimeout, P_LATERAL_PID.slew};
// Precise Angular
inline const pahlib::ControllerSettings P_angular_PID = {
    P_ANGULAR_PID.kP, P_ANGULAR_PID.kI, P_ANGULAR_PID.kD, P_ANGULAR_PID.kF, P_ANGULAR_PID.antiWindup, 
    P_ANGULAR_PID.smallError, P_ANGULAR_PID.smallTimeout, P_ANGULAR_PID.largeError, 
    P_ANGULAR_PID.largeTimeout, P_ANGULAR_PID.slew};

// --- Distance Sensor Offsets ---
// Distance from the actual sensor reading point to the center of the robot in inches. 
inline constexpr float DS_FRONT_X = -5.15f;
inline constexpr float DS_BACK_X  = 5.25f;
inline constexpr float DS_LEFT_X  = -6.0f;
inline constexpr float DS_RIGHT_X = 6.0f;

inline constexpr float DS_FRONT_Y = 5.3f;
inline constexpr float DS_BACK_Y  = -4.5f;
inline constexpr float DS_LEFT_Y  = -2.5f;
inline constexpr float DS_RIGHT_Y = -2.5f;

// Lateral gain schedule (distance in inches)
struct LateralSchedule {
    std::vector<float> distances = {2.0f, 5.0f, 10.0f, 20.0f};
    std::vector<pahlib::Chassis::PIDGains> gains = {
        {7.0f, 0.0f, 11.0f, 0.0f},   // 2 inches
        {8.0f, 0.0f, 10.5f, 0.0f},   // 5 inches
        {10.0f, 0.0f, 9.5f, 0.0f},   // 10 inches
        {12.0f, 0.0f, 10.0f, 0.0f}    // 20 inches
    };
};

// Angular gain schedule (angle in degrees)
struct AngularSchedule {
    std::vector<float> angles = {15.0f, 45.0f, 90.0f, 180.0f};
    std::vector<pahlib::Chassis::PIDGains> gains = {
        {1.5f, 0.0f, 25.0f, 0.0f},  // 15 degrees
        {2.0f, 0.0f, 25.0f, 0.0f},  // 45 degrees
        {4.0f, 0.0f, 25.0f, 0.0f},  // 90 degrees
        {5.0f, 0.0f, 25.0f, 0.0f}   // 180 degrees
    };
};

#endif