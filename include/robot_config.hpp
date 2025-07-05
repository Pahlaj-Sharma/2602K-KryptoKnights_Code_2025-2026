#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

// --- Motor Ports ---
// Define motor port numbers for clarity and easy modification.
// Negative numbers usually indicate a motor's direction needs to be reversed.
#define PORT_RIGHT_MOTOR_1 11
#define PORT_RIGHT_MOTOR_2 13
#define PORT_RIGHT_MOTOR_3 12
#define PORT_LEFT_MOTOR_1  -14 // Note: these will be negated again in Robot constructor for reversal
#define PORT_LEFT_MOTOR_2  -15 // Note: these will be negated again in Robot constructor for reversal
#define PORT_LEFT_MOTOR_3  -16 // Note: these will be negated again in Robot constructor for reversal

// --- Sensor Ports ---
#define PORT_IMU                 2  // Inertial Measurement Unit
#define PORT_HORIZONTAL_ENCODER -3 // Horizontal tracking wheel encoder (negative for reversed direction)
#define PORT_VERTICAL_ENCODER    17 // Vertical tracking wheel encoder
#define PORT_AUTON_SELECTOR_POT  6  // Potentiometer for autonomous routine selection
#define PORT_TEAM_SELECTOR_POT   7  // Potentiometer used for team selection (treated as a switch)

// Define distance sensor ports. Adjust if they are on different ports or used differently.
#define PORT_DISTANCE_RIGHT      3
#define PORT_DISTANCE_LEFT       3
#define PORT_DISTANCE_FRONT      3
#define PORT_DISTANCE_BACK       3

// --- Drivetrain Constants (in inches/RPM as appropriate) ---
#define TRACK_WIDTH 11.875     // Distance between the centers of the left and right wheels in inches
#define WHEEL_DIAMETER 2.75    // Diameter of your drivetrain wheels (e.g., 2.75" Omniwheels)
#define MOTOR_RPM 450          // Max effective RPM of your drivetrain motors (e.g., 600 RPM blue motors with 1.33:1 external gearing = 450 RPM)
#define DRIVETRAIN_GEARING 2.0 // External gearing ratio applied to the drivetrain (e.g., 2.0 for 2:1 speed increase)

// --- Odometry Tracking Wheel Offsets ---
// Offsets from the robot's center to the tracking wheel in inches.
// Negative for horizontal means it's behind the center, or adjust sign based on orientation.
#define HORIZONTAL_TRACKING_OFFSET -5.25
#define VERTICAL_TRACKING_OFFSET 0.0625

// --- PID Controller Settings for LemLib Chassis ---
// These constants define how the robot's movement and turning are controlled.

// Lateral PID (for straight movement and path following)
#define LATERAL_KP 7.0         // Proportional constant
#define LATERAL_KI 0.0         // Integral constant (often zero for simple control)
#define LATERAL_KD 9.0         // Derivative constant
#define LATERAL_SETTLE_ERR 3   // Error threshold (in inches) to consider the robot settled
#define LATERAL_SETTLE_TIME 100 // Time (ms) robot must be within settle_err
#define LATERAL_TIMEOUT 3      // Total timeout (seconds) for movement
#define LATERAL_EXIT_ERR 500   // Error threshold (in inches) to exit early if taking too long
#define LATERAL_EXIT_TIME 15   // Time (ms) robot must be within exit_err to exit early
#define LATERAL_MIN_SPEED 15   // Minimum speed the robot will move at

// Angular PID (for turning)
#define ANGULAR_KP 2.0         // Proportional constant
#define ANGULAR_KI 0.0         // Integral constant
#define ANGULAR_KD 16.0        // Derivative constant
#define ANGULAR_SETTLE_ERR 3   // Error threshold (in degrees) to consider the robot settled
#define ANGULAR_SETTLE_TIME 100 // Time (ms) robot must be within settle_err
#define ANGULAR_TIMEOUT 3      // Total timeout (seconds) for turn
#define ANGULAR_EXIT_ERR 500   // Error threshold (in degrees) to exit early if taking too long
#define ANGULAR_EXIT_TIME 0    // Time (ms) robot must be within exit_err to exit early
#define ANGULAR_MIN_SPEED 0    // Minimum speed the robot will turn at

// --- Autonomous Selector Constants ---
#define AUTON_POT_MAX_VALUE 330 // Maximum expected potentiometer value for 10 autons (0-330)
#define AUTON_DIVISOR 33        // Value range for each autonomous segment (330 / 10 autons = 33)
#define NUM_AUTONS 10           // Total number of autonomous routines available

// --- Team Selector Constants ---
#define TEAM_RED_MAX_VALUE 165  // Potentiometer value threshold for selecting the RED team.
                                // Values <= 165 are RED, > 165 are BLUE.

#endif