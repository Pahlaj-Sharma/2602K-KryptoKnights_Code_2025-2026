/**
 * Project: 2602K-RobotCode
 * Author: Pahlaj Sharma
 * Date Created: June 14, 2025
 * Current Version: 3.10
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
#include <map>

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

    pros::lcd::initialize(); // Initialize the VEX LCD
    chassis.calibrate();     // Calibrate the odometry sensors (IMU, encoders)

    // Create a task to continuously print robot pose, robot temp, battery, auton to the brain screen
    pros::Task update_robot_info([&]() {
        int count = 0;
        while (true) {
            // Print robot location to the brain screen
            if (pros::competition::is_autonomous()){
                // Print current pose only in autonomous mode
                pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
                pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
                pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);
            }
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
        std::string autonName = autons.at(selectedAuton).first;

        // Chaneg to line 1 and 2 after remove printing current pose
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Auton: %s", autonName.c_str());
        controller.print(2, 0, "%s", autonName.c_str());

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

// REMOVE WHEN DONE TUNING
// --- PID Tuning Components ---
// These components are for live PID tuning and should be removed once tuning is complete.
pros::Rotation rot_kp(1);
pros::Rotation rot_ki(2);
pros::Rotation rot_kd(3);

void tunePID() {
    // Store initial PID and rotation sensor values
    const int initial_rot_kp_pos = rot_kp.get_position();
    const int initial_rot_ki_pos = rot_ki.get_position();
    const int initial_rot_kd_pos = rot_kd.get_position();

    // Scaling factors for PID tuning
    constexpr float KP_SCALE = 0.1f;
    constexpr float KI_SCALE = 0.01f;
    constexpr float KD_SCALE = 0.1f;

    while (true) {
        // Calculate deltas from rotation sensors
        float delta_kp = (rot_kp.get_position() - initial_rot_kp_pos) * KP_SCALE;
        float delta_ki = (rot_ki.get_position() - initial_rot_ki_pos) * KI_SCALE;
        float delta_kd = (rot_kd.get_position() - initial_rot_kd_pos) * KD_SCALE;

        // Update lateral PID values
        chassis.lateralPID.kP += delta_kp;
        chassis.lateralPID.kI += delta_ki;
        chassis.lateralPID.kD += delta_kd;

        // Display PID values on controller
        controller.print(0, 0, "kP: %.3f", chassis.lateralPID.kP);
        controller.print(1, 0, "kI: %.3f", chassis.lateralPID.kI);
        controller.print(2, 0, "kD: %.3f", chassis.lateralPID.kD);

        // Run test movement if button A is pressed
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            controller.rumble("-");
            chassis.calibrate();
            chassis.setPose(0, 0, 0);
            pros::delay(100);
            chassis.moveToPoint(0, 24, 10000); // Example test movement
            chassis.waitUntilDone();
            controller.rumble(".");
            controller.clear();
        }
        pros::delay(50);
    }
}
