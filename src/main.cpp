/**
 * Project: 2602K-RobotCode
 * Author: Pahlaj Sharma
 * Date Created: June 14, 2025
 * Current Version: 4.01
 * Last Updated: Oct 2, 2025
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
Motor left_front(PORT_LEFT_FRONT, MotorGears::blue);
Motor left_middle(PORT_LEFT_MIDDLE, MotorGears::green);
Motor left_back(PORT_LEFT_BACK, MotorGears::blue);
Motor right_front(PORT_RIGHT_FRONT, MotorGears::blue);
Motor right_middle(PORT_RIGHT_MIDDLE, MotorGears::green);
Motor right_back(PORT_RIGHT_BACK, MotorGears::blue);

Motor left_pto(PORT_LEFT_PTO, MotorGears::blue);
Motor right_pto(PORT_RIGHT_PTO, MotorGears::blue);
Motor score_motor(PORT_SCORE_MOTOR, MotorGears::blue);

MotorGroup left_motors({PORT_LEFT_FRONT, PORT_LEFT_MIDDLE, PORT_LEFT_BACK, PORT_LEFT_PTO}, MotorGears::blue);
MotorGroup right_motors({PORT_RIGHT_FRONT, PORT_RIGHT_MIDDLE, PORT_RIGHT_BACK, PORT_RIGHT_PTO}, MotorGears::blue);

// --- Sensors ---
Rotation vertical_encoder(PORT_VERTICAL_ENCODER);
Rotation horizontal_encoder(PORT_HORIZONTAL_ENCODER);
adi::Potentiometer autonSelector(PORT_AUTON_SELECTOR_POT);
Distance rightDistance(PORT_DISTANCE_RIGHT);
Distance leftDistance(PORT_DISTANCE_LEFT);
Distance frontDistance(PORT_DISTANCE_FRONT);
Distance backDistance(PORT_DISTANCE_BACK);
adi::DigitalOut pto(PORT_PTO);
ScalarIMU inertial(PORT_IMU, IMU_SCALER);
adi::DigitalOut matchLoad(PORT_MATCH_LOAD);
adi::DigitalOut centerGoal(PORT_CENTER_GOAL);
adi::DigitalOut doublePark(PORT_DOUBLE_PARK);
adi::DigitalOut antenne(PORT_ANTENNE);

// --- Drivetrain Setup ---
Drivetrain drivetrain(
    &left_motors, &right_motors, TRACK_WIDTH,
    Omniwheel::NEW_2, WHEEL_RPM, HORIZONTAL_DRIFT
);

TrackingWheel vertical_tracking_wheel(
    &vertical_encoder, Omniwheel::NEW_2, VERTICAL_TRACKING_OFFSET
);
TrackingWheel horizontal_tracking_wheel(
    &horizontal_encoder, Omniwheel::NEW_2, HORIZONTAL_TRACKING_OFFSET
);

// FIXXX
OdomSensors sensors(
    &vertical_tracking_wheel, nullptr,
    &horizontal_tracking_wheel, nullptr, 
    &inertial
);

ExpoDriveCurve drive_curve(5, 20, 1.02);
ExpoDriveCurve steer_curve(5, 35, 1.02);

Chassis chassis(
    drivetrain, lateral_PID, angular_PID, sensors,
    &drive_curve, &steer_curve
);

// --- Autonomous Routines ---
std::map<int, std::pair<std::string, std::function<void()>>> autons = {
    {0, {"name", auton1}}, {1, {"name", auton1}}, {2, {"name", auton2}},
    {3, {"name", auton3}}, {4, {"name", auton4}}, {5, {"name", auton5}},
    {6, {"name", auton6}}, {7, {"name", auton7}}, {8, {"name", auton8}},
    {9, {"name", auton9}}, {10, {"name", auton10}}
}; // Maps auton number to name and function

// CHANGE FOR ANY AUTON
// 1 = skills
// 2 = right
// 3 = left

int selectedAuton = 3;
bool ptoState = true;
int antiJam = 1;

// --- Initialization ---
void initialize() {
    left_motors.set_gearing(MotorGears::green, 1);
    right_motors.set_gearing(MotorGears::green, 1);

    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    vertical_encoder.reset_position();

    chassis.calibrate();
    controller.clear();

    toggle_pto(true);

    // Background task to update robot info on screen and controller
    Task update_robot_info([&]() {
        int count = 0;
        while (true) {
            if (competition::is_autonomous()) {
                screen::print(E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
                screen::print(E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
                screen::print(E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);
            }

            if (count % 200 == 0) {
                controller.print(0, 0, "Temp: %.1f", std::max(left_middle.get_temperature(), right_middle.get_temperature()));
            }
            count++;
            delay(25);
        }
    });
/*
    Task anti_jam([&]() {
        while (true) {
        if ((std::fabs(right_pto.get_actual_velocity()) < 50) && (controller.get_digital(E_CONTROLLER_DIGITAL_R1) || controller.get_digital(E_CONTROLLER_DIGITAL_L1))) {
            pros::delay(400);
            if ((std::fabs(right_pto.get_actual_velocity()) < 50) && (controller.get_digital(E_CONTROLLER_DIGITAL_R1) || controller.get_digital(E_CONTROLLER_DIGITAL_L1))) antiJam = -1;
            else antiJam = 1;
        } pros::delay(90);}
    });
*/
    std::vector<bool> devices_connected = {
        inertial.is_installed(), rightDistance.is_installed(), leftDistance.is_installed(), frontDistance.is_installed(),
        backDistance.is_installed(), left_front.is_installed(), left_middle.is_installed(), left_back.is_installed(),
        right_front.is_installed(), right_middle.is_installed(), right_back.is_installed(), left_pto.is_installed(),
        right_pto.is_installed(), vertical_encoder.is_installed(), horizontal_encoder.is_installed(), score_motor.is_installed()
    };
    std::vector<std::string> device_names = {
        "IMU", "R_Dist", "L_Dist", "F_Dist", "B_Dist", "L_Front", "L_Middle",
        "L_Back", "R_Front", "R_Middle", "R_Back", "L_PTO", "R_PTO", "V_Tracker", "H_Tracker", "Score_Motor"
    };
    if (!std::all_of(devices_connected.begin(), devices_connected.end(), [](bool v) { return v; })) {
        int line = 4;
        for (size_t i = 0; i < devices_connected.size(); i++) {
            if (!devices_connected[i]) {
                screen::print(E_TEXT_MEDIUM, line, "%s is not connected", device_names[i].c_str());
                line++;
            }
        }
    }
}

void disabled() {
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
}

void competition_initialize() {
    controller.clear();
    // Select auton using potentiometer before match starts
    while (competition::is_disabled()) {
        // Read the potentiometer value to select auton
        selectedAuton = static_cast<int>(autonSelector.get_angle() / (330.0 / autons.size()));
        // Print the selected auton on the screen and controller
        if (autons.count(selectedAuton)) {
            std::string autonName = autons.at(selectedAuton).first;
            screen::print(E_TEXT_MEDIUM, 3, "Auton: %s", autonName.c_str());
            controller.print(0, 5, "%s", autonName.c_str());
        }
        delay(200);
    }
}

void autonomous() {
    vertical_encoder.reset_position();
    // turn to brake if not consistent
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST); //left_motors.set_brake_mode(E_MOTOR_BRAKE_COAST, 1);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST); //right_motors.set_brake_mode(E_MOTOR_BRAKE_COAST, 1);

    // 3 is for left

    if (autons.count(selectedAuton)) autons.at(selectedAuton).second();
    else autons.at(4).second();
    
}

void opcontrol() {
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    
    bool intakeToggle = false; 
    bool scoreToggle = false;
    bool centerToggle = false;
    bool antenneState = false;
    bool loadToggle = false;

    while (true) {
        // Drive Control
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
        
        // Intake Preroller Toggle
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
            intakeToggle = !intakeToggle;
            toggle_preroller(intakeToggle);
        }
        
        // Intake Score Control
        if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
            scoreToggle = true;
            toggle_score(scoreToggle);
        } if (controller.get_digital_new_release(E_CONTROLLER_DIGITAL_R1)) {
            scoreToggle = false;
            toggle_score(scoreToggle);
        }
         
        // Intake Center Goal
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
            centerToggle = !centerToggle;
            centerGoal.set_value(centerToggle);
            toggle_score(centerToggle, 80, 70);
        }

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)){
            ptoState = !ptoState;
            toggle_pto(ptoState);
        }

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
            antenneState = !antenneState;
            antenne.set_value(antenneState);
        }

        if (controller.get_digital(E_CONTROLLER_DIGITAL_L2)) {
            toggle_score(true, -80, -60);
        } if (controller.get_digital_new_release(E_CONTROLLER_DIGITAL_L2)){
            toggle_score(false);
        }

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
            loadToggle = !loadToggle;
            matchLoad.set_value(loadToggle);
        }

        pros::delay(10);
    }
}

// --- PID Tuning (Remove When Done) ---
/*
Rotation rot_kp(1), rot_ki(2), rot_kd(3);

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

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
            controller.rumble("-");
            chassis.calibrate();
            chassis.setPose(0, 0, 0);
            delay(100);
            chassis.moveTo(0, 24, 10000);
            chassis.waitUntilDone();
            controller.rumble(".");
            controller.clear();
        }
        delay(50);
    }
}
*/