#include "main.h" // PROS main header
#include "lemlib/api.hpp" // LemLib API for odometry and chassis control
#include "lemlib/util.hpp"
#include "pros/misc.hpp"
#include "robot_config.hpp"
#include "autons.hpp"
#include "subsystems.hpp"
#include <map>
#include <string>

// --- Controller Definition ---
// Initializes the primary VEX V5 controller connected to the robot.
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Motor Definitions ---
// MotorGroup is a LemLib class that allows you to control multiple motors as a single unit.
// Using constants from robot_config.hpp for port numbers.
pros::MotorGroup right_motors({PORT_RIGHT_MOTOR_1, PORT_RIGHT_MOTOR_2, PORT_RIGHT_MOTOR_3}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({PORT_LEFT_MOTOR_1, PORT_LEFT_MOTOR_2, PORT_LEFT_MOTOR_3}, pros::MotorGearset::blue);

// --- Sensor Definitions ---
// Using constants from robot_config.hpp for port numbers.
pros::Imu imu(PORT_IMU);
pros::Rotation horizontal_encoder(PORT_HORIZONTAL_ENCODER);
pros::Rotation vertical_encoder(PORT_VERTICAL_ENCODER);
pros::adi::Potentiometer autonSelector(PORT_AUTON_SELECTOR_POT);
pros::adi::Potentiometer teamSelector(PORT_TEAM_SELECTOR_POT);
pros::Distance rightDistance(PORT_DISTANCE_RIGHT);
pros::Distance leftDistance(PORT_DISTANCE_LEFT);
pros::Distance frontDistance(PORT_DISTANCE_FRONT);
pros::Distance backDistance(PORT_DISTANCE_BACK);

// --- LemLib Definitions ---
// Drivetrain configuration, using constants from robot_config.hpp
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, TRACK_WIDTH, lemlib::Omniwheel::NEW_275, WHEEL_RPM, HORIZONTAL_DRIFT);

// Odometry Tracking Wheel configurations, using constants from robot_config.hpp
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, HORIZONTAL_TRACKING_OFFSET);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, VERTICAL_TRACKING_OFFSET);

// Odometry Sensors configuration
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// PID Controller Settings, using constants from robot_config.hpp
lemlib::ControllerSettings lateral_controller(LATERAL_KP, LATERAL_KI, LATERAL_KD, LATERAL_ANTI_WINDUP, LATERAL_SML_ERR, LATERAL_SML_TIMEOUT, LATERAL_LRG_ERR, LATERAL_LRG_TIMEOUT, LATERAL_SLEW);
lemlib::ControllerSettings angular_controller(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, ANGULAR_ANTI_WINDUP, ANGULAR_SML_ERR, ANGULAR_SML_TIMEOUT, ANGULAR_LRG_ERR, ANGULAR_LRG_TIMEOUT, ANGULAR_SLEW);

// Input Curve for throttle/steer input during driver control
lemlib::ExpoDriveCurve drive_curve(3, 20, 1.02);

// Chassis definition: Integrates all LemLib components
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &drive_curve, &drive_curve);

// Global Variables
int selectedAuton = 1;
std::string teamtype = "RED";

//@brief Runs initialization code.
void initialize() {
    // Set brake modes for drivetrain motors
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // Reset odometry encoder positions
    horizontal_encoder.reset_position();
    vertical_encoder.reset_position();

    pros::lcd::initialize(); // Initialize the VEX LCD (for basic prints)
    chassis.calibrate();     // Calibrate the LemLib odometry sensors (IMU, encoders)
    
    // Create a task to continuously print robot pose (X, Y, Theta) to the brain screen
    pros::Task update_odom([&]() {
        while (true) {
            // Print robot location to the brain screen
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);      // X coordinate
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);      // Y coordinate
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta); // Heading (angle)

            pros::delay(25); // Small delay to save resources and prevent blocking
        }
    });
    // Create a task to continuously print robot temp, battery, auton to the controller screen
    pros::Task robot_info([&]() {
        while (true) {
            // Print robot location to the brain screen
            controller.print(0, 0, "Battery: %f", pros::battery::get_capacity()); // Print Current Battery Level
            controller.print(1, 0, "DT Temp: %f", ((left_motors.get_temperature() + right_motors.get_temperature()) / 2)); // Print Avg temp of motors

            pros::delay(5000); // Delay to save resources and prevent blocking
        }
    });
}

// Runs while the robot is in the disabled state.
void disabled() {
    //
}

// Runs after initialize(), and before autonomous() or opcontrol().
void competition_initialize() {
    pros::screen::erase(); // Clear the screen initially for a clean display

    // --- Optimization 1 & 2: Initialize map once, outside the loop ---
    // This map defines the names of your autonomous routines.
    std::map<int, std::string> auton_map = {
        {1, "Auton1"}, {2, "Auton2"}, {3, "Auton3"}, {4, "Auton4"}, {5, "Auton5"},
        {6, "Auton6"}, {7, "Auton7"}, {8, "Auton8"}, {9, "Auton9"}, {10, "Auton10"},
    };

    while (true) {
        // Read potentiometer values to determine selection
        int potValue = autonSelector.get_value();
        
        // Determine selected autonomous routine
        selectedAuton = (potValue < 0) ? 1 : (potValue > 329) ? 10 : (potValue / 33) + 1;

        // Determine team type based on teamSelector potentiometer's angle
        // Assuming TEAM_RED_MAX_VALUE or similar constant is used, or replace 165 with your constant.
        teamtype = (teamSelector.get_angle() >= 0 && teamSelector.get_angle() <= 165) ? "RED" : "BLUE";

        // Display selected autonomous routine description on the screen
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "%s", ("Autonomous: " + auton_map[selectedAuton]).c_str());
        
        // Display selected team type
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "%s", ("Team: " + teamtype).c_str()); 

        // Display on Contoller screen
        controller.print(2, 0, "%s :: %s",teamtype.c_str(),  auton_map[selectedAuton].c_str());

        // Add a small delay to control update rate and prevent CPU hogging.
        pros::delay(200);
    }
}

// Runs the user autonomous code.
void autonomous() {
    // Select and run the chosen autonomous routine based on 'autonSelect' variable.
    // (0 = blue side auton, 1 = red side auton, or specific routine index)
    switch (selectedAuton) {
        case 1:
            auton1(); break;
        case 2:
            auton2(); break;
        case 3:
            auton3(); break;
        case 4:
            auton4(); break;
        case 5:
            auton5(); break;
        case 6:
            auton6(); break;
        case 7:
            auton7(); break;
        case 8:
            auton8(); break;
        case 9:
            auton9(); break;
        case 10:
            auton10(); break;
        default:
            auton1(); break;
    }
}


//Runs the operator control code.
void opcontrol() {
    while (true) {
        // --- Driving Control (Arcade Style) ---
        // Get joystick values for left Y-axis (forward/backward) and right X-axis (turning)
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Control the chassis using arcade drive
        // 'leftY' controls forward/backward, 'rightX' controls turning
        chassis.arcade(leftY, rightX);

        // --- Delay for resources ---
        // A small delay to yield control to other PROS tasks and reduce CPU usage.
        pros::delay(25);
    }
}
