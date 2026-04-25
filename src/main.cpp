/**
 * Project: 2602K-RobotCode
 * Author: Pahlaj Sharma
 * Date Created: June 14, 2025
 * Current Version: 4.01
 * Last Updated: Mar 2, 2026
 *
 * @copyright (c) 2025, @Pahlaj-Sharma
 * All rights reserved.
 *
 **/

#include "main.h"
#include "pahlib/api.hpp"
#include "robot_config.hpp"
#include "autons.hpp"
#include "subsystems.hpp"
#include <map>

using namespace pros;
using namespace pahlib;

// Initializes the primary controller connected to the robot
Controller controller(E_CONTROLLER_MASTER);

// --- Motor Definitions ---
// Define individual motors for the drivetrain
Motor left_front(PORT_LEFT_FRONT, MotorGears::blue);
Motor left_middle(PORT_LEFT_MIDDLE, MotorGears::blue);
Motor left_back(PORT_LEFT_BACK, MotorGears::blue);
Motor right_front(PORT_RIGHT_FRONT, MotorGears::blue);
Motor right_middle(PORT_RIGHT_MIDDLE, MotorGears::blue);
Motor right_back(PORT_RIGHT_BACK, MotorGears::blue);

// Define motors for intake and scoring mechanisms
Motor intake_motor(PORT_INTAKE_MOTOR, MotorGears::blue);
Motor score_motor(PORT_SCORE_MOTOR, MotorGears::blue);

// Group motors for easier control
MotorGroup left_motors({PORT_LEFT_FRONT, PORT_LEFT_MIDDLE, PORT_LEFT_BACK}, MotorGears::blue);
MotorGroup right_motors({PORT_RIGHT_FRONT, PORT_RIGHT_MIDDLE, PORT_RIGHT_BACK}, MotorGears::blue);

// --- Sensors ---
// Encoders for tracking wheel positions
Rotation vertical_encoder(PORT_VERTICAL_ENCODER);
Rotation horizontal_encoder(PORT_HORIZONTAL_ENCODER);
adi::DigitalIn autonSelector(PORT_AUTON_SELECTOR); // Potentiometer for selecting autonomous routine

// Distance sensors for obstacle detection
Distance rightDistance(PORT_DISTANCE_RIGHT);
Distance leftDistance(PORT_DISTANCE_LEFT);
Distance frontDistance(PORT_DISTANCE_FRONT);
Distance backDistance(PORT_DISTANCE_BACK);

// Inertial Measurement Unit for orientation
ScalarIMU inertial(PORT_IMU, IMU_SCALER);

// Pneumatic solenoids for various mechanisms
adi::DigitalOut matchLoad(PORT_MATCH_LOAD);
adi::DigitalOut centerGoal(PORT_CENTER_GOAL);
adi::DigitalOut doublePark(PORT_DOUBLE_PARK);
adi::DigitalOut antenne(PORT_ANTENNE);
adi::DigitalOut score(6);
adi::DigitalOut descore(8);
//adi::DigitalOut lowGoal(7);

// RCL (Relative Coordinate Localization) sensors for positioning
RclSensor front_rcl(&frontDistance, DS_FRONT_X, DS_FRONT_Y, 0.0, 10.0);
RclSensor right_rcl(&rightDistance, DS_RIGHT_X, DS_RIGHT_Y, 90.0, 10.0);
RclSensor back_rcl(&backDistance, DS_BACK_X, DS_BACK_Y, 180.0, 10.0);
RclSensor left_rcl(&leftDistance, DS_LEFT_X, DS_LEFT_Y, 270.0, 10.0);

// Define circular obstacles on the field (loaders)
inline Circle_Obstacle redUpLoader(-67.5, 46.5, 3);
inline Circle_Obstacle redDownLoader(-67.5, -46.5, 3);
inline Circle_Obstacle blueUpLoader(67.5, 46.5, 3);
inline Circle_Obstacle blueDownLoader(67.5, -46.5, 3);

// Define circular obstacles for goals
inline Circle_Obstacle upLongGoalLeft(-21, 47.5, 4);
inline Circle_Obstacle upLongGoalRight(21, 47.5, 4);
inline Circle_Obstacle downLongGoalLeft(-21, -47.5, 4);
inline Circle_Obstacle downLongGoalRight(21, -47.5, 4);
inline Circle_Obstacle centerGoals(0, 0, 5);

// Line obstacle to disable certain areas during autonomous
inline Line_Obstacle disableLine(0, FIELD_NEG_HALF_LENGTH, 0, FIELD_HALF_LENGTH);

// --- Drivetrain Setup ---
// Configure the drivetrain with motor groups, track width, wheel type, etc.
Drivetrain drivetrain(
    &left_motors, &right_motors, TRACK_WIDTH,
    Omniwheel::NEW_325, WHEEL_RPM, HORIZONTAL_DRIFT
);

// Tracking wheels for odometry
TrackingWheel vertical_tracking_wheel(
    &vertical_encoder, 1.965739f, VERTICAL_TRACKING_OFFSET
);
TrackingWheel horizontal_tracking_wheel(
    &horizontal_encoder, 1.97048458f, HORIZONTAL_TRACKING_OFFSET
);

// Odometry sensors setup
OdomSensors sensors(
    &vertical_tracking_wheel, nullptr,
    &horizontal_tracking_wheel, nullptr,
    &inertial
);

// Drive and steer curves for exponential control
ExpoDriveCurve drive_curve(5, 25, 1.05);
ExpoDriveCurve steer_curve(5, 5, 1.01);

// Chassis controller with PID and sensors
Chassis chassis(
    drivetrain, lateral_PID, angular_PID, sensors,
    &drive_curve, &steer_curve
);

// RCL tracking for position resets
RclTracking reset(&chassis, 20, true, 0.5, 4.0, 10.0, 2.0, 20);

// --- Autonomous Routines ---
// Map of autonomous routines: key is auton number, value is pair of name and function
std::map<int, std::pair<std::string, std::function<void()>>> autons = {
    {1, {"Skills", auton1}}, {2, {"Counter SAWP", auton2}}, {3, {"Fast Right 4", auton3}},
    {4, {"Right 7", auton4}}, {5, {"Right 9 Tech", auton5}}, {6, {"Right 9 Split", auton6}},
    {7, {"Fast Left 4", auton7}}, {8, {"Left 7", auton8}}, {9, {"Left 7 Split", auton9}}, 
    {10, {"Left 7 Center First", auton10}},
};

// Function to update selected autonomous based on potentiometer angle

int selectedAuton = 1;
bool antiJamEnable = true;

// --- Initialization ---
// Called when the robot initializes
void initialize() {
    // Set brake modes for motors
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);

    // Start tracking and calibrate chassis
    reset.startTracking();
    chassis.calibrate();

    // Reset encoder positions
    vertical_encoder.reset_position();
    horizontal_encoder.reset_position();
    controller.clear();

    // Background task to update robot info on screen and controller
    Task update_robot_info([&]() {
        int count = 0;
        while (true) {
            // Display position during autonomous
            if (competition::is_autonomous()) {
                screen::print(E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
                screen::print(E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
                screen::print(E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);
            }

            // Periodically display temperatures and selected auton on controller
            if (count % 100 == 0) {
                double hottest_drive = std::max({
                    left_front.get_temperature(), left_middle.get_temperature(), left_back.get_temperature(),
                    right_front.get_temperature(), right_middle.get_temperature(), right_back.get_temperature()
                });

                double hottest_intake = std::max({
                    intake_motor.get_temperature(),
                    score_motor.get_temperature()
                });
                controller.print(0, 0, "D:%.0f I:%.0f A:%d", hottest_drive, hottest_intake, selectedAuton);
            }
            count++;
            delay(25);
        }
    });

    // Check device connections
    std::vector<bool> devices_connected = {
        inertial.is_installed(), left_front.is_installed(), left_middle.is_installed(), left_back.is_installed(),
        right_front.is_installed(), right_middle.is_installed(), right_back.is_installed(), intake_motor.is_installed(),
        score_motor.is_installed(), vertical_encoder.is_installed(), horizontal_encoder.is_installed(), score_motor.is_installed(),
        rightDistance.is_installed(), leftDistance.is_installed(), frontDistance.is_installed(), backDistance.is_installed()
    };

    std::vector<int> devices_port = {
        inertial.get_port(), left_front.get_port(), left_middle.get_port(), left_back.get_port(),
        right_front.get_port(), right_middle.get_port(), right_back.get_port(), intake_motor.get_port(),
        score_motor.get_port(), vertical_encoder.get_port(), horizontal_encoder.get_port(), score_motor.get_port(),
        rightDistance.get_port(), leftDistance.get_port(), frontDistance.get_port(), backDistance.get_port()
    };

    std::vector<std::string> device_names = {
        "IMU", "L_Front", "L_Middle",
        "L_Back", "R_Front", "R_Middle", "R_Back", "Intake_Motor", "Score_Motor", "V_Tracker", "H_Tracker", "Score_Motor",
        "Right_Distance", "Left_Distance", "Front_Distance", "Back_Distance"
    };

    // Display disconnected devices on screen
    if (!std::all_of(devices_connected.begin(), devices_connected.end(), [](bool v) { return v; })) {
        int line = 5;
        for (size_t i = 0; i < devices_connected.size(); i++) {
            if (!devices_connected[i]) {
                screen::print(E_TEXT_MEDIUM, line, "%s not connected, port %d", device_names[i].c_str(), devices_port[i]);
                line++;
            }
        }
    }
}

// Called when the robot is disabled
void disabled() {
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
}

// Called during competition initialization
void competition_initialize() {
    controller.clear();
    doublePark.set_value(false); // turn on

    std::vector<std::string> auton_setup = {
        "Left of Park; Right Side DT; Facing 90", 
        "Middle of Park; Right Side DT; Facing 180",
        "Right of Park; Left Side DT; Facing 90",
        "Right of Park; Left Side DT; Facing 90", 
        "Right of Park; Left Side DT; Facing 90", 
        "Right of Park; Left Side DT; Facing 90",
        "Left of Park; Right Side DT; Facing 90", 
        "Left of Park; Right Side DT; Facing 90", 
        "Left of Park; Right Side DT; Facing 180",
        "Left of Park; Right Side DT; Facing 90"
    };

    // Select auton using potentiometer before match starts
    while (competition::is_disabled()) {
        if (autonSelector.get_new_press()) {
            selectedAuton++;
            if (selectedAuton > autons.size()) selectedAuton = 1;
        }
        std::string autonName = autons.at(selectedAuton).first;
        screen::print(E_TEXT_MEDIUM, 3, "                            ");
        screen::print(E_TEXT_MEDIUM, 3, "Auton: %s", autonName.c_str());
        screen::print(E_TEXT_MEDIUM, 4, "                                                 ");
        screen::print(E_TEXT_MEDIUM, 4, "Setup: %s", auton_setup[selectedAuton - 1].c_str());
        delay(50);
    }
}

// Autonomous period function
void autonomous() {
    /* Order of autons:
    1. Skills
    2. Counter SAWP
    3. Fast Right 4
    4. Right 7
    5. Right 9 Tech
    6. Right 9 Split
    7. Fast Left 4
    8. Left 7
    9. Left 7 Split
    10. Left 7 Center First */

    // Reset encoders
    vertical_encoder.reset_position();
    horizontal_encoder.reset_position();

    // Set brake modes
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    doublePark.set_value(false); // turn on

    // Run selected autonomous routine
    if (autons.count(selectedAuton)) autons.at(selectedAuton).second();
    else autons.at(1).second();
}

// Operator control function
void opcontrol() {
    // Set brake modes
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);

    // Initialize toggle states
    bool intakeToggle, scoreToggle, centerToggle, antenneState, loadToggle, descoreState = false;
    uint32_t r1_press_time = 0;
    bool r1_active = false;
    bool timer_expired = false;
    uint32_t l1_start_time = 0;
    bool l1_active = false;

    // Initial setup for pneumatics and tracking
    doublePark.set_value(true); // turn off odom
    centerGoal.set_value(false);
    score.set_value(false);
    reset.stopTracking();
    toggle_score(false);

    while (true) {
        // Drive Control: Arcade drive using left Y and right X
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // Intake Preroller Toggle: Y button toggles preroller
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
            toggle_score(false);
            intakeToggle = !intakeToggle;
            toggle_preroller(intakeToggle);
        }

        // Intake Score Control: R1 button hold for scoring
        if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
            scoreToggle = true;
            score.set_value(scoreToggle);
            toggle_score(scoreToggle);
        } if (controller.get_digital_new_release(E_CONTROLLER_DIGITAL_R1)) {
            scoreToggle = false;
            score.set_value(scoreToggle);
            toggle_score(scoreToggle);
        }

        /*
            if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
                centerToggle = true;
                centerGoal.set_value(true);
                toggle_score(true, -50, -80, false);

                uint32_t start_p1 = pros::millis();
                while (pros::millis() - start_p1 < 200) {
                    if (!controller.get_digital(E_CONTROLLER_DIGITAL_L1)) {
                        // SHUTDOWN
                        centerGoal.set_value(false);
                        toggle_score(false); // Uses default 110s, but multiplies by 0
                        centerToggle = false;
                        break;
                    }
                    pros::delay(10);
                }
                if (centerToggle) {

                    uint32_t start_p2 = pros::millis();
                    while (pros::millis() - start_p2 < 160) {
                        if (!controller.get_digital(E_CONTROLLER_DIGITAL_L1)) {
                            centerGoal.set_value(false);
                            toggle_score(false);
                            centerToggle = false;
                            break;
                        }
                        pros::delay(10);
                    }
                }

                if (centerToggle) {
                    toggle_score(true, 110, -110);
                }
                } if (controller.get_digital_new_release(E_CONTROLLER_DIGITAL_L1)) {
                    centerToggle = false;
                    centerGoal.set_value(false);
                    toggle_score(false);
                }
        */

        // Middle goal scoring: L1 button with timed phases
        if (controller.get_digital(E_CONTROLLER_DIGITAL_L1)) {
            if (!l1_active) {
                l1_active = true;
                l1_start_time = pros::millis();
            }

            uint32_t held_time = pros::millis() - l1_start_time;

            if (held_time < 200) {
                centerGoal.set_value(false);
                toggle_score(true, -50, -80, false);
            } else {
                centerGoal.set_value(true);
                toggle_score(true, 110, -110, true);
            }
        } else {
            if (l1_active) {
                centerGoal.set_value(false);
                toggle_score(false);
                l1_active = false;
            }
        }

        // Antenna toggle: R2 button
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
            antenneState = !antenneState;
            antenne.set_value(antenneState);
        }

        // Descore toggle: A button
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
            descoreState = !descoreState;
            descore.set_value(descoreState);
        }

        // Low goal scoring: L2 button
        if (controller.get_digital(E_CONTROLLER_DIGITAL_L2)) {
            toggle_score(true, -80, -60);
        } if (controller.get_digital_new_release(E_CONTROLLER_DIGITAL_L2)) {
            toggle_score(false);
        }

        // Match load toggle: Right arrow button
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
            loadToggle = !loadToggle;
            matchLoad.set_value(loadToggle);
        }

        // Automated sequence for loading: Up arrow button
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
            chassis.tank(20, 30, true);
            toggle_score(false);
            antenne.set_value(true);
            antenneState = true;
            delay(200);
            chassis.tank(-40, -40, true);
            delay(200);
            chassis.tank(65, 65, true);
            delay(200);
            chassis.tank(75, 85, true);
            toggle_preroller(true, 115);
            delay(1500);
            matchLoad.set_value(true);
            intakeToggle = true;
            chassis.tank(60, 70, true);
            delay(250);
            matchLoad.set_value(false);
            loadToggle = false;
            delay(500);
            chassis.tank(0, 0, true);
        }

        // Clear bottom sequence: Down arrow button
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
            chassis.tank(20, 30, true);
            toggle_score(false);
            antenne.set_value(true);
            antenneState = true;
            delay(200);
            chassis.tank(-40, -40, true);
            delay(200);
            chassis.tank(65, 65, true);
            delay(200);
            chassis.tank(85, 75, true);
            toggle_preroller(true, 115);
            delay(1500);
            matchLoad.set_value(true);
            intakeToggle = true;
            chassis.tank(70, 60, true);
            delay(250);
            matchLoad.set_value(false);
            loadToggle = false;
            delay(500);
            chassis.tank(0, 0, true);
        }

        delay(10);
    }
}