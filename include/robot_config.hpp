#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP
#include "pahlib/chassis/chassis.hpp"
// --- Motor Ports ---
// Define motor port numbers for clarity and easy modification.
// A negative port number (e.g., -14) indicates that the motor's direction should be reversed.

inline constexpr int PORT_RIGHT_MOTOR_FRONT = -11;
inline constexpr int PORT_RIGHT_MOTOR_MIDDLE = 13;
inline constexpr int PORT_RIGHT_MOTOR_BACK = -12;
inline constexpr int PORT_LEFT_MOTOR_FRONT  = 14;
inline constexpr int PORT_LEFT_MOTOR_MIDDLE  = -15;
inline constexpr int PORT_LEFT_MOTOR_BACK  = 16;
inline constexpr int PORT_LEFT_PTO      = 20;
inline constexpr int PORT_RIGHT_PTO     = -21;

// --- Sensor Ports ---
inline constexpr int PORT_IMU                = 2;  // Inertial Measurement Unit
//inline constexpr int PORT_HORIZONTAL_ENCODER = 3;  // Horizontal tracking wheel encoder (negative for reversed direction)
inline constexpr int PORT_VERTICAL_ENCODER   = 17; // Vertical tracking wheel encoder
inline constexpr int PORT_AUTON_SELECTOR_POT = 6;  // Potentiometer for autonomous routine selection
inline constexpr int PORT_TEAM_SELECTOR_POT  = 7;  // Potentiometer for team selection
inline constexpr int PORT_PTO_DIGITAL_OUT    = 8;  // Digital output for PTO control
inline constexpr int PORT_DISTANCE_RIGHT     = 4;
inline constexpr int PORT_DISTANCE_LEFT      = 18;
inline constexpr int PORT_DISTANCE_FRONT     = 5;
inline constexpr int PORT_DISTANCE_BACK      = 19;

// --- Drivetrain Constants (in inches/RPM as appropriate) ---
inline constexpr float TRACK_WIDTH      = 11.55; // Distance between the centers of the left and right wheels in inches
inline constexpr int    WHEEL_RPM        = 450;   // Max effective RPM of the drivetrain (e.g., 600 RPM blue motors with 1.33:1 external gearing)
inline constexpr float HORIZONTAL_DRIFT = 8.0;   // Horizontal drift in inches, used for odometry calculations
inline constexpr float IMU_SCALER       = 1.0;   // Custom IMU scaling factor, adjust based on IMU's calibration

// --- Odometry Tracking Wheel Offsets ---
// Offsets from the robot's center to the tracking wheel in inches.
//inline constexpr float HORIZONTAL_TRACKING_OFFSET = 0.0;
inline constexpr float VERTICAL_TRACKING_OFFSET   = 0.0;

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
    7.0, 0.0, 9.0, 0.0, 3, 1, 100, 2, 500, 15
};
// Custom "Fast" Lateral PID constants
inline constexpr PIDConstants F_LATERAL_PID {
    7.0, 0.0, 9.0, 0.0, 3, 2, 100, 3, 500, 15
};
// Custom "Precise" Lateral PID constants
inline constexpr PIDConstants P_LATERAL_PID {
    7.0, 0.0, 9.0, 0.0, 3, 1, 100, 2, 500, 15
};
// Default Angular PID constants
inline constexpr PIDConstants ANGULAR_PID {
    2.0, 0.0, 16.0, 0.0, 3, 1, 100, 3, 500, 0
};
// Custom "Fast" Angular PID constants
inline constexpr PIDConstants F_ANGULAR_PID {
    2.0, 0.0, 16.0, 0.0, 3, 2, 100, 4, 500, 0
};
// Custom "Precise" Angular PID constants
inline constexpr PIDConstants P_ANGULAR_PID {
    2.0, 0.0, 16.0, 0.0, 3, 1, 100, 2, 500, 0
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
inline constexpr float DS_FRONT_CENTER = 5.5;
inline constexpr float DS_BACK_CENTER  = 1.375;
inline constexpr float DS_LEFT_CENTER  = 1.75;
inline constexpr float DS_RIGHT_CENTER = 1.75;

#endif