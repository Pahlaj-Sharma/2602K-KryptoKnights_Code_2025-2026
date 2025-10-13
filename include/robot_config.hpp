#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP
#include "pahlib/chassis/chassis.hpp"
// --- Motor Ports ---
// Define motor port numbers for clarity and easy modification.

inline constexpr int PORT_RIGHT_FRONT  = 8;
inline constexpr int PORT_RIGHT_MIDDLE = -17 ;
inline constexpr int PORT_RIGHT_BACK   = 10;
inline constexpr int PORT_LEFT_FRONT   = -2;
inline constexpr int PORT_LEFT_MIDDLE  = 11;
inline constexpr int PORT_LEFT_BACK    = -1;
inline constexpr int PORT_LEFT_PTO     = 3;
inline constexpr int PORT_RIGHT_PTO    = -7;
inline constexpr int PORT_SCORE_MOTOR  = 15;

// --- Sensor Ports ---
inline constexpr int PORT_IMU                = 21; // Inertial Measurement Unit
inline constexpr int PORT_HORIZONTAL_ENCODER = 16; // Horizontal tracking wheel encoder
inline constexpr int PORT_VERTICAL_ENCODER   = 9;  // Vertical tracking wheel encoder
inline constexpr int PORT_AUTON_SELECTOR_POT = 6;  // Potentiometer for autonomous routine selection
inline constexpr int PORT_TEAM_SELECTOR_POT  = 7;  // Potentiometer for team selection
inline constexpr int PORT_PTO_DIGITAL_OUT    = 8;  // Digital output for PTO control
inline constexpr int PORT_DISTANCE_RIGHT     = 7;
inline constexpr int PORT_DISTANCE_LEFT      = 18;
inline constexpr int PORT_DISTANCE_FRONT     = 5;
inline constexpr int PORT_DISTANCE_BACK      = 19;
inline constexpr int PORT_MATCH_LOAD         = 20;
inline constexpr int PORT_CENTER_GOAL        = 9;
inline constexpr int PORT_DOUBLE_PARK        = 4;
inline constexpr int PORT_ANTENNE            = 3;

// --- Drivetrain Constants (in inches/RPM as appropriate) ---
inline constexpr float TRACK_WIDTH      = 11.55f; // Distance between the centers of the left and right wheels in inches
inline constexpr int   WHEEL_RPM        = 450;   // Max effective RPM of the drivetrain (e.g., 600 RPM blue motors with 1.33:1 external gearing)
inline constexpr float HORIZONTAL_DRIFT = 2.0f;   // Horizontal drift in inches, used for odometry calculations
inline constexpr float IMU_SCALER       = 1.010445f;   // Custom IMU scaling factor, adjust based on IMU's calibration

// --- Odometry Tracking Wheel Offsets ---
// Offsets from the robot's center to the tracking wheel in inches.
inline constexpr float VERTICAL_TRACKING_OFFSET   = 0.0f;
inline constexpr float HORIZONTAL_TRACKING_OFFSET   = -0.75f;
inline constexpr int HYSTERESIS_CYCLES = 3; // Robot must be past target for 3 cycles (30ms), increase if undershooting

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
    7.0f, 0.0f, 9.0f, 0.0f, 3, 1, 100, 2, 500, 20
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
    5.0f, 0.0f, 16.0f, 0.0f, 3, 1, 100, 3, 500, 0
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
inline constexpr float DS_FRONT_CENTER = 5.53f;
inline constexpr float DS_BACK_CENTER  = 1.37f;
inline constexpr float DS_LEFT_CENTER  = 1.75f;
inline constexpr float DS_RIGHT_CENTER = 1.75f;

// Lateral gain schedule (distance in inches)
struct LateralSchedule {
    std::vector<float> distances = {5.0f, 10.0f, 20.0f, 40.0f};
    std::vector<pahlib::Chassis::PIDGains> gains = {
        {8.0f, 0.0f, 0.5f, 0.0f},   // 5 inches
        {6.0f, 0.0f, 0.4f, 0.0f},   // 10 inches
        {4.5f, 0.0f, 0.3f, 0.0f},   // 20 inches
        {3.0f, 0.0f, 0.2f, 0.0f}    // 40 inches
    };
};

// Angular gain schedule (angle in degrees)
struct AngularSchedule {
    std::vector<float> angles = {15.0f, 45.0f, 90.0f, 180.0f};
    std::vector<pahlib::Chassis::PIDGains> gains = {
        {2.5f, 0.0f, 0.15f, 0.0f},  // 15 degrees
        {2.0f, 0.0f, 0.12f, 0.0f},  // 45 degrees
        {1.5f, 0.0f, 0.10f, 0.0f},  // 90 degrees
        {1.0f, 0.0f, 0.08f, 0.0f}   // 180 degrees
    };
};

#endif