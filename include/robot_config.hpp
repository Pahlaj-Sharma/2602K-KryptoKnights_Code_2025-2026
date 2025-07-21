#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

// --- Motor Ports ---
// Define motor port numbers for clarity and easy modification.
// Negative numbers usually indicate a motor's direction needs to be reversed.
#define PORT_RIGHT_MOTOR_1 11
#define PORT_RIGHT_MOTOR_2 13
#define PORT_RIGHT_MOTOR_3 12
#define PORT_LEFT_MOTOR_1  -14
#define PORT_LEFT_MOTOR_2  -15
#define PORT_LEFT_MOTOR_3  -16

// --- Sensor Ports ---
#define PORT_IMU                 2  // Inertial Measurement Unit
#define PORT_HORIZONTAL_ENCODER  -3 // Horizontal tracking wheel encoder (negative for reversed direction)
#define PORT_VERTICAL_ENCODER    17 // Vertical tracking wheel encoder
#define PORT_AUTON_SELECTOR_POT  6  // Potentiometer for autonomous routine selection
#define PORT_TEAM_SELECTOR_POT   7  // Potentiometer used for team selection (treated as a switch)

// Define distance sensor ports.
#define PORT_DISTANCE_RIGHT      4
#define PORT_DISTANCE_LEFT       18
#define PORT_DISTANCE_FRONT      5
#define PORT_DISTANCE_BACK       19

// --- Drivetrain Constants (in inches/RPM as appropriate) ---
#define TRACK_WIDTH 11.875       // Distance between the centers of the left and right wheels in inches
#define WHEEL_DIAMETER 2.75      // Diameter of your drivetrain wheels (e.g., 2.75" Omniwheels)
#define WHEEL_RPM 450            // Max effective RPM of your drivetrain motors (e.g., 600 RPM blue motors with 1.33:1 external gearing = 450 RPM)
#define HORIZONTAL_DRIFT 2.0     // External gearing ratio applied to the drivetrain (e.g., 2.0 for 2:1 speed increase)

// --- Odometry Tracking Wheel Offsets ---
// Offsets from the robot's center to the tracking wheel in inches.
// Negative for horizontal means it's behind the center, or adjust sign based on orientation.
#define HORIZONTAL_TRACKING_OFFSET -5.25
#define VERTICAL_TRACKING_OFFSET 0

// --- PID Controller Settings for LemLib Chassis ---
// These constants define how the robot's movement and turning are controlled.

// Lateral PID (for straight movement and path following)
#define LATERAL_KP 7.0           // Proportional constant
#define LATERAL_KI 0.0           // Integral constant (often zero for simple control)
#define LATERAL_KD 9.0           // Derivative constant
#define LATERAL_ANTI_WINDUP 3    // Error threshold (in inches) to consider the robot settled
#define LATERAL_SML_ERR 1        // Smallest error
#define LATERAL_SML_TIMEOUT 100  // Smalles timeout in calculation
#define LATERAL_LRG_ERR 3        // Largest error
#define LATERAL_LRG_TIMEOUT 500  // Largest timeout in calculation
#define LATERAL_SLEW 15          // Slew
// Custom PIDs
#define F_LATERAL_KP 7.0           // Proportional constant
#define F_LATERAL_KI 0.0           // Integral constant (often zero for simple control)
#define F_LATERAL_KD 9.0           // Derivative constant
#define P_LATERAL_KP 7.0           // Proportional constant
#define P_LATERAL_KI 0.0           // Integral constant (often zero for simple control)
#define P_LATERAL_KD 9.0           // Derivative constant

// Angular PID (for turning)
#define ANGULAR_KP 2.0           // Proportional constant
#define ANGULAR_KI 0.0           // Integral constant
#define ANGULAR_KD 16.0          // Derivative constant
#define ANGULAR_ANTI_WINDUP 3    // Error threshold (in degrees) to consider the robot settled
#define ANGULAR_SML_ERR 1        // Smallest error
#define ANGULAR_SML_TIMEOUT 100  // Smalles timeout in calculation
#define ANGULAR_LRG_ERR 3        // Largest error
#define ANGULAR_LRG_TIMEOUT 500  // Largest timeout in calculation
#define ANGULAR_SLEW 0           // Slew
// Custom PIDs
#define F_ANGULAR_KP 2.0           // Proportional constant
#define F_ANGULAR_KI 0.0           // Integral constant
#define F_ANGULAR_KD 16.0          // Derivative constant
#define P_ANGULAR_KP 2.0           // Proportional constant
#define P_ANGULAR_KI 0.0           // Integral constant
#define P_ANGULAR_KD 16.0          // Derivative constant

// --- Distance Sensor Offsets ---
// Distance from the actual distance sensor reading point to the closest physical edge of the robot
// in that direction. E.g., if your front sensor is 1 inch behind the absolute front of the robot.
#define DS_FRONT_INDENT 5.5
#define DS_BACK_INDENT  1.375
#define DS_LEFT_INDENT  1.75
#define DS_RIGHT_INDENT 1.75
#define DS_FRONT_CENTER 5.5
#define DS_BACK_CENTER  1.375
#define DS_LEFT_CENTER  1.75
#define DS_RIGHT_CENTER 1.75

#endif