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
#include "pahlib/api.hpp" 
#include "robot_config.hpp"
#include "autons.hpp"
#include "subsystems.hpp"
#include <map>

using namespace pahlib;

// Initializes the primary controller connected to the robot
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Motor Definitions ---
pros::Motor left_front(PORT_LEFT_MOTOR_1, pros::v5::MotorGears::blue);
pros::Motor left_middle(PORT_LEFT_MOTOR_2, pros::v5::MotorGears::green);
pros::Motor left_back(-PORT_LEFT_MOTOR_3, pros::v5::MotorGears::blue); // Negative sign reverses motor direction
pros::Motor right_front(PORT_RIGHT_MOTOR_1, pros::v5::MotorGears::blue);
pros::Motor right_middle(-PORT_RIGHT_MOTOR_2, pros::v5::MotorGears::green); // Negative sign reverses motor direction
pros::Motor right_back(PORT_RIGHT_MOTOR_3, pros::v5::MotorGears::blue);
pros::Motor left_pto(PORT_LEFT_PTO, pros::v5::MotorGears::blue);
pros::Motor right_pto(-PORT_RIGHT_PTO, pros::v5::MotorGears::blue); // Negative sign reverses motor direction

pros::MotorGroup left_motors({left_front}); // Motor groups for easier control
pros::MotorGroup right_motors({right_front});

// --- Sensors ---
pros::Rotation horizontal_encoder(PORT_HORIZONTAL_ENCODER);
pros::Rotation vertical_encoder(PORT_VERTICAL_ENCODER);
pros::adi::Potentiometer autonSelector(PORT_AUTON_SELECTOR_POT);
pros::Distance rightDistance(PORT_DISTANCE_RIGHT);
pros::Distance leftDistance(PORT_DISTANCE_LEFT);
pros::Distance frontDistance(PORT_DISTANCE_FRONT);
pros::Distance backDistance(PORT_DISTANCE_BACK);
pros::adi::DigitalOut pto(PORT_PTO_DIGITAL_OUT);
ScalarIMU inertial(PORT_IMU, IMU_SCALER);

// --- Drivetrain Setup ---
Drivetrain drivetrain(
    &left_motors, &right_motors, TRACK_WIDTH,
    Omniwheel::NEW_275, WHEEL_RPM, HORIZONTAL_DRIFT
);

TrackingWheel horizontal_tracking_wheel(
    &horizontal_encoder, Omniwheel::NEW_2, HORIZONTAL_TRACKING_OFFSET
);
TrackingWheel vertical_tracking_wheel(
    &vertical_encoder, Omniwheel::NEW_2, VERTICAL_TRACKING_OFFSET
);

OdomSensors sensors(
    &vertical_tracking_wheel, nullptr,
    &horizontal_tracking_wheel, nullptr,
    &inertial
);

ExpoDriveCurve drive_curve(5, 20, 1.02);

Chassis chassis(
    drivetrain, lateral_PID, angular_PID, sensors,
    &drive_curve, &drive_curve
);

// --- Autonomous Routines ---
std::map<int, std::pair<std::string, std::function<void()>>> autons = {
    {0, {"name", auton1}}, {1, {"name", auton1}}, {2, {"name", auton2}},
    {3, {"name", auton3}}, {4, {"name", auton4}}, {5, {"name", auton5}},
    {6, {"name", auton6}}, {7, {"name", auton7}}, {8, {"name", auton8}},
    {9, {"name", auton9}}, {10, {"name", auton10}}
}; // Maps auton number to name and function

int selectedAuton = 1;
bool ptoState = false;

// --- Initialization ---
void initialize() {
    left_motors.append(left_middle); left_motors.append(left_back);
    right_motors.append(right_middle); right_motors.append(right_back);

    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    horizontal_encoder.reset_position();
    vertical_encoder.reset_position();

    pros::lcd::initialize();
    chassis.calibrate();

    // Background task to update robot info on screen and controller
    pros::Task update_robot_info([&]() {
        int count = 0;
        while (true) {
            if (pros::competition::is_autonomous()) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
                pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
                pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);
            }
            if (count % 200 == 0) {
                controller.print(0, 0, "Battery: %.1f", pros::battery::get_capacity());
                controller.print(1, 0, "DT Temp: %.1f",
                    std::max(left_front.get_temperature(), right_front.get_temperature()));
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
    pros::screen::erase();
    // Select auton using potentiometer before match starts
    while (pros::competition::is_disabled()) {
        selectedAuton = static_cast<int>(autonSelector.get_angle() / (330.0 / autons.size()));
        std::string autonName = autons.at(selectedAuton).first;
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

    if (autons.count(selectedAuton)) {
        autons.at(selectedAuton).second();
    } else {
        autons.at(0).second();
    }
}

void opcontrol() {
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            toggle_pto(!ptoState);
        }
        pros::delay(10);
    }
}

// --- PID Tuning (Remove When Done) ---
pros::Rotation rot_kp(1), rot_ki(2), rot_kd(3);

void tunePID() {
    const int initial_rot_kp_pos = rot_kp.get_position();
    const int initial_rot_ki_pos = rot_ki.get_position();
    const int initial_rot_kd_pos = rot_kd.get_position();

    constexpr float KP_SCALE = 0.1f, KI_SCALE = 0.01f, KD_SCALE = 0.1f;

    while (true) {
        float delta_kp = (rot_kp.get_position() - initial_rot_kp_pos) * KP_SCALE;
        float delta_ki = (rot_ki.get_position() - initial_rot_ki_pos) * KI_SCALE;
        float delta_kd = (rot_kd.get_position() - initial_rot_kd_pos) * KD_SCALE;

        chassis.lateralPID.kP += delta_kp;
        chassis.lateralPID.kI += delta_ki;
        chassis.lateralPID.kD += delta_kd;

        controller.print(0, 0, "kP: %.3f", chassis.lateralPID.kP);
        controller.print(1, 0, "kI: %.3f", chassis.lateralPID.kI);
        controller.print(2, 0, "kD: %.3f", chassis.lateralPID.kD);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            controller.rumble("-");
            chassis.calibrate();
            chassis.setPose(0, 0, 0);
            pros::delay(100);
            chassis.moveToPoint(0, 24, 10000);
            chassis.waitUntilDone();
            controller.rumble(".");
            controller.clear();
        }
        pros::delay(50);
    }
}
