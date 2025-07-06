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

// --- Robot Physical Dimensions (approximate, for odometry resets) ---
// Distance from robot's center to its front/back edge
#define ROBOT_HALF_LENGTH_INCHES 7.6875 // Example: if your robot is 18 inches long
// Distance from robot's center to its left/right edge
#define ROBOT_HALF_WIDTH_INCHES  7.0 // Example: if your robot is 18 inches wide

// --- Distance Sensor Offsets ---
// Distance from the actual distance sensor reading point to the closest physical edge of the robot
// in that direction. E.g., if your front sensor is 1 inch behind the absolute front of the robot.
#define DS_FRONT_OFFSET_INCHES 5.5
#define DS_BACK_OFFSET_INCHES  1.375
#define DS_LEFT_OFFSET_INCHES  1.75
#define DS_RIGHT_OFFSET_INCHES 1.75

// --- Field Dimensions (assuming origin (0,0) is at the center of a 12x12 foot field) ---
#define FIELD_HALF_LENGTH_INCHES 72.0 // Half of 12 feet (144 inches)
#define FIELD_HALF_WIDTH_INCHES  72.0 // Half of 12 feet (144 inches)

// --- Wall Detection Threshold ---
// Max distance (in inches) for a sensor reading to be considered "against a wall"
#define WALL_DETECTION_THRESHOLD_INCHES 12.0 // Adjust based on your sensor accuracy and robot speed

#endif