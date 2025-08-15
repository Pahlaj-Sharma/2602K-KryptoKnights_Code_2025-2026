/**
 * Project: 2602K-RobotCode
 * Author: Pahlaj Sharma
 * Date Created: June 14, 2025
 * Current Version: 3.01
 * Last Updated: Aug 7, 2025
 *
 * Copyright (c) 2025, Pahlaj Sharma.
 * All rights reserved.
 *
 **/

#include "main.h" // PROS main header
#include "lemlib/api.hpp"
#include "robot_config.hpp"
#include "autons.hpp"
#include "subsystems.hpp"
#include "functions.hpp"
#include <cmath>
#include <map>
#include <string>

// --- Controller Definition ---
// Initializes the primary controller connected to the robot
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Motor Definitions ---
// MotorGroup is a class that allows to control multiple motors as a single unit
// Using constants from robot_config.hpp for port numbers
pros::Motor left_front(PORT_LEFT_MOTOR_1, pros::v5::MotorGears::blue);
pros::Motor left_middle(PORT_LEFT_MOTOR_2, pros::v5::MotorGears::green);
pros::Motor left_back(PORT_LEFT_MOTOR_3, pros::v5::MotorGears::blue);
pros::Motor right_front(PORT_RIGHT_MOTOR_1, pros::v5::MotorGears::blue);
pros::Motor right_middle(PORT_RIGHT_MOTOR_2, pros::v5::MotorGears::green);
pros::Motor right_back(PORT_RIGHT_MOTOR_3, pros::v5::MotorGears::blue);
pros::Motor left_pto(PORT_LEFT_PTO, pros::v5::MotorGears::blue);
pros::Motor right_pto(PORT_RIGHT_PTO, pros::v5::MotorGears::blue);

pros::MotorGroup left_motors({left_front});
pros::MotorGroup right_motors({right_front});

// --- Sensor Definitions ---
// Using constants from robot_config.hpp for port numbers
pros::Rotation horizontal_encoder(PORT_HORIZONTAL_ENCODER);
pros::Rotation vertical_encoder(PORT_VERTICAL_ENCODER);
pros::adi::Potentiometer autonSelector(PORT_AUTON_SELECTOR_POT);
pros::adi::Potentiometer teamSelector(PORT_TEAM_SELECTOR_POT);
pros::Distance rightDistance(PORT_DISTANCE_RIGHT);
pros::Distance leftDistance(PORT_DISTANCE_LEFT);
pros::Distance frontDistance(PORT_DISTANCE_FRONT);
pros::Distance backDistance(PORT_DISTANCE_BACK);
pros::adi::DigitalOut pto(PORT_PTO_DIGITAL_OUT);
ScalarIMU inertial(PORT_IMU, IMU_SCALER);

// --- LemLib Definitions ---
// Drivetrain configuration, using constants from robot_config.hpp
lemlib::Drivetrain drivetrain(
    &left_motors, // left motor group
    &right_motors, // right motor group
    TRACK_WIDTH, // track width
    lemlib::Omniwheel::NEW_275, // wheel type
    WHEEL_RPM, // wheel RPM
    HORIZONTAL_DRIFT // horizontal drift
);

// Odometry Tracking Wheel configurations, using constants from robot_config.hpp
lemlib::TrackingWheel horizontal_tracking_wheel(
    &horizontal_encoder, // encoder
    lemlib::Omniwheel::NEW_2, // wheel type
    HORIZONTAL_TRACKING_OFFSET // offset
);
lemlib::TrackingWheel vertical_tracking_wheel(
    &vertical_encoder, // encoder
    lemlib::Omniwheel::NEW_2, // wheel type
    VERTICAL_TRACKING_OFFSET // offset
);

// Odometry Sensors configuration
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel
    nullptr, // vertical tracking wheel 2
    &horizontal_tracking_wheel, // horizontal tracking wheel
    nullptr, // horizontal tracking wheel 2
    &inertial // inertial sensor
);

// Input Curve for throttle/steer input during driver control
lemlib::ExpoDriveCurve drive_curve(5, 20, 1.02);

// Chassis definition: Integrates all components
lemlib::Chassis chassis(
    drivetrain, // drivetrain
    lateral_PID, // lateral controller
    angular_PID, // angular controller
    sensors, // odometry sensors
    &drive_curve, // throttle curve
    &drive_curve // steer curve
);

// Global Variables
std::map<int, std::pair<std::string, std::function<void()>>> autons = {
    {0, {"name", auton1}},
    {1, {"name", auton1}},
    {2, {"name", auton2}},
    {3, {"name", auton3}},
    {4, {"name", auton4}},
    {5, {"name", auton5}},
    {6, {"name", auton6}},
    {7, {"name", auton7}},
    {8, {"name", auton8}},
    {9, {"name", auton9}},
    {10, {"name", auton10}}
};

int selectedAuton = 1;
std::string teamType = "RED";
bool ptoState = false; // PTO state: true = intake, false = drivetrain

void initialize() {
    // Set brake modes for drivetrain and PTO motors
    left_motors.append(left_middle); left_motors.append(left_back);
    right_motors.append(right_middle); right_motors.append(right_back);
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // Reset odometry encoder positions
    horizontal_encoder.reset_position();
    vertical_encoder.reset_position();

    pros::lcd::initialize(); // Initialize the VEX LCD (for basic prints)
    chassis.calibrate();     // Calibrate the odometry sensors (IMU, encoders)

    // Create a task to continuously print robot pose, robot temp, battery, auton to the brain screen
    pros::Task update_robot_info([&]() {
        int count = 0;
        while (true) {
            // Print robot location to the brain screen
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);

            if (count % 200 == 0) {
                // Print current battery level and motor temps to controller
                controller.print(0, 0, "Battery: %.1f", pros::battery::get_capacity());
                controller.print(1, 0, "DT Temp: %.1f", std::max(left_front.get_temperature(), right_front.get_temperature()));
            }
            count++;
            pros::delay(25);
        }
    });
}

void disabled() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void competition_initialize() {
    pros::screen::erase(); // Clear the screen initially for a clean display
    while (pros::competition::is_disabled()) {
        // Determine selected autonomous routine
        selectedAuton = static_cast<int>(autonSelector.get_angle() / (330.0 / autons.size()));
        // Determine team type based on teamSelector potentiometer's angle
        teamType = (teamSelector.get_angle() <= 165) ? "RED" : "BLUE";
        std::string autonName = autons.at(selectedAuton).first;

        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Auton: %s", autonName.c_str());
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Team: %s", teamType.c_str());
        controller.print(2, 0, "%s :: %s", teamType.c_str(), autonName.c_str());

        pros::delay(200);
    }
}

void autonomous() {
    horizontal_encoder.reset_position();
    vertical_encoder.reset_position();
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    // Select and run the chosen autonomous routine based on 'selectedAuton' variable.
    if (autons.count(selectedAuton)) {
        autons.at(selectedAuton).second();
    } else {
        // Default to a safe routine if something goes wrong
        autons.at(0).second();
    }
}

void opcontrol() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        // Get joystick values for left Y-axis (forward/backward) and right X-axis (turning)
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Control the chassis using arcade drive
        chassis.arcade(leftY, rightX);

        // Add condition for PTO toggle
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            toggle_pto(!ptoState);
        }

        pros::delay(10);
    }
}