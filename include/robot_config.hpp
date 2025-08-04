#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

// --- Motor Ports ---
// Define motor port numbers for clarity and easy modification.
// Negative numbers usually indicate a motor's direction needs to be reversed.
inline constexpr int PORT_RIGHT_MOTOR_1 = 11;
inline constexpr int PORT_RIGHT_MOTOR_2 = 13;
inline constexpr int PORT_RIGHT_MOTOR_3 = 12;
inline constexpr int PORT_LEFT_MOTOR_1  = -14;
inline constexpr int PORT_LEFT_MOTOR_2  = -15;
inline constexpr int PORT_LEFT_MOTOR_3  = -16;
inline constexpr int PORT_LEFT_PTO      = 20;
inline constexpr int PORT_RIGHT_PTO     = 21;

// --- Sensor Ports ---
inline constexpr int PORT_IMU                 = 2;  // Inertial Measurement Unit
inline constexpr int PORT_HORIZONTAL_ENCODER  = -3; // Horizontal tracking wheel encoder (negative for reversed direction)
inline constexpr int PORT_VERTICAL_ENCODER    = 17; // Vertical tracking wheel encoder
inline constexpr int PORT_AUTON_SELECTOR_POT  = 6;  // Potentiometer for autonomous routine selection
inline constexpr int PORT_TEAM_SELECTOR_POT   = 7;  // Potentiometer used for team selection (treated as a switch)
inline constexpr int PORT_PTO_DIGITAL_OUT     = 8;  // Digital output for PTO control
inline constexpr int PORT_DISTANCE_RIGHT      = 4;
inline constexpr int PORT_DISTANCE_LEFT       = 18;
inline constexpr int PORT_DISTANCE_FRONT      = 5;
inline constexpr int PORT_DISTANCE_BACK       = 19;

// --- Drivetrain Constants (in inches/RPM as appropriate) ---
inline constexpr double TRACK_WIDTH      = 11.55; // Distance between the centers of the left and right wheels in inches
inline constexpr int    WHEEL_RPM        = 450;   // Max effective RPM of your drivetrain motors (e.g., 600 RPM blue motors with 1.33:1 external gearing = 450 RPM)
inline constexpr double HORIZONTAL_DRIFT = 2.0;   // Horizontal drift in inches, used for odometry calculations
inline constexpr double IMU_SCALER       = 1.0;   // Custom IMU scaling factor, adjust based on your IMU's calibration

// --- Odometry Tracking Wheel Offsets ---
// Offsets from the robot's center to the tracking wheel in inches.
// Negative for horizontal means it's behind the center, or adjust sign based on orientation.
inline constexpr double HORIZONTAL_TRACKING_OFFSET = 0.0;
inline constexpr double VERTICAL_TRACKING_OFFSET   = 0.0;

// --- PID Controller Settings for LemLib Chassis ---
// These are good candidates for a struct to group them together
// especially since you have multiple sets of constants (F_LATERAL, P_LATERAL).
struct PIDConstants {
    double kP;
    double kI;
    double kD;
    int antiWindup;
    int smallError;
    int smallTimeout;
    int largeError;
    int largeTimeout;
    int slew;
};

// Lateral PID
inline constexpr PIDConstants LATERAL_PID {
    7.0, 0.0, 9.0, 3, 1, 100, 2, 500, 15
};
// Custon Lateral PIDs
inline constexpr PIDConstants F_LATERAL_PID {
    7.0, 0.0, 9.0
};

inline constexpr PIDConstants P_LATERAL_PID {
    7.0, 0.0, 9.0
};
// Angular PID
inline constexpr PIDConstants ANGULAR_PID {
    2.0, 0.0, 16.0, 3, 1, 100, 2, 500, 0
};
// Custom Angular PIDs
inline constexpr PIDConstants F_ANGULAR_PID {
    2.0, 0.0, 16.0
};
inline constexpr PIDConstants P_ANGULAR_PID {
    2.0, 0.0, 16.0
};

// --- Distance Sensor Offsets ---
// Distance from the actual distance sensor reading point to the center of the robot in inches.
inline constexpr double DS_FRONT_CENTER = 5.5;
inline constexpr double DS_BACK_CENTER  = 1.375;
inline constexpr double DS_LEFT_CENTER  = 1.75;
inline constexpr double DS_RIGHT_CENTER = 1.75;

#endif