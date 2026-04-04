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
#include "pahlib/RclTracking.hpp"
#include <map>

using namespace pros;
using namespace pahlib;

// Initializes the primary controller connected to the robot
Controller controller(E_CONTROLLER_MASTER);

// --- Motor Definitions ---
Motor left_front(PORT_LEFT_FRONT, MotorGears::blue);
Motor left_middle(PORT_LEFT_MIDDLE, MotorGears::blue);
Motor left_back(PORT_LEFT_BACK, MotorGears::blue);
Motor right_front(PORT_RIGHT_FRONT, MotorGears::blue);
Motor right_middle(PORT_RIGHT_MIDDLE, MotorGears::blue);
Motor right_back(PORT_RIGHT_BACK, MotorGears::blue);

Motor intake_motor(PORT_INTAKE_MOTOR, MotorGears::blue);
Motor score_motor(PORT_SCORE_MOTOR, MotorGears::blue);

MotorGroup left_motors({PORT_LEFT_FRONT, PORT_LEFT_MIDDLE, PORT_LEFT_BACK}, MotorGears::blue);
MotorGroup right_motors({PORT_RIGHT_FRONT, PORT_RIGHT_MIDDLE, PORT_RIGHT_BACK}, MotorGears::blue);

// --- Sensors ---
Rotation vertical_encoder(PORT_VERTICAL_ENCODER);
Rotation horizontal_encoder(PORT_HORIZONTAL_ENCODER);
adi::Potentiometer autonSelector(PORT_AUTON_SELECTOR_POT, E_ADI_POT_V2);
Distance rightDistance(PORT_DISTANCE_RIGHT);
Distance leftDistance(PORT_DISTANCE_LEFT);
Distance frontDistance(PORT_DISTANCE_FRONT);
Distance backDistance(PORT_DISTANCE_BACK);
ScalarIMU inertial(PORT_IMU, IMU_SCALER);
adi::DigitalOut matchLoad(PORT_MATCH_LOAD);
adi::DigitalOut centerGoal(PORT_CENTER_GOAL);
adi::DigitalOut doublePark(PORT_DOUBLE_PARK);
adi::DigitalOut antenne(PORT_ANTENNE);
adi::DigitalOut score(6);
adi::DigitalOut descore(8);

// Rcl setup
RclSensor front_rcl(&frontDistance, DS_FRONT_X, DS_FRONT_Y, 0.0, 10.0);
RclSensor right_rcl(&rightDistance, DS_RIGHT_X, DS_RIGHT_Y, 90.0, 10.0);
RclSensor back_rcl(&backDistance, DS_BACK_X, DS_BACK_Y, 180.0, 10.0);
RclSensor left_rcl(&leftDistance, DS_LEFT_X, DS_LEFT_Y, 270.0, 10.0);

// loaders
inline Circle_Obstacle redUpLoader(-67.5, 46.5, 3);
inline Circle_Obstacle redDownLoader(-67.5, -46.5, 3);
inline Circle_Obstacle blueUpLoader(67.5, 46.5, 3);
inline Circle_Obstacle blueDownLoader(67.5, -46.5, 3);

// legs
inline Circle_Obstacle upLongGoalLeft(-21, 47.5, 4);
inline Circle_Obstacle upLongGoalRight(21, 47.5, 4);
inline Circle_Obstacle downLongGoalLeft(-21, -47.5, 4);
inline Circle_Obstacle downLongGoalRight(21, -47.5, 4);
inline Circle_Obstacle centerGoals(0, 0, 5);

// Disable Line for the autonomous period
inline Line_Obstacle disableLine(0, FIELD_NEG_HALF_LENGTH, 0, FIELD_HALF_LENGTH);

// --- Drivetrain Setup ---
Drivetrain drivetrain(
    &left_motors, &right_motors, TRACK_WIDTH,
    Omniwheel::NEW_325, WHEEL_RPM, HORIZONTAL_DRIFT
);

TrackingWheel vertical_tracking_wheel(
    &vertical_encoder, 1.965739f, VERTICAL_TRACKING_OFFSET
);
TrackingWheel horizontal_tracking_wheel(
    &horizontal_encoder, 1.97048458f,  HORIZONTAL_TRACKING_OFFSET
);

OdomSensors sensors(
    &vertical_tracking_wheel, nullptr,
    &horizontal_tracking_wheel, nullptr, 
    &inertial
);

ExpoDriveCurve drive_curve(5, 25, 1.05);
ExpoDriveCurve steer_curve(5, 5, 1.01);

Chassis chassis(
    drivetrain, lateral_PID, angular_PID, sensors,
    &drive_curve, &steer_curve
);

RclTracking reset(&chassis, 20, true, 0.5, 4.0, 10.0, 2.0, 20);

// --- Autonomous Routines ---
std::map<int, std::pair<std::string, std::function<void()>>> autons = {
    {0, {"SKILLS", auton1}}, {1, {"SKILLS", auton1}}, {2, {"SAWP", auton2}},
    {3, {"RIGHT-7", auton3}}, {4, {"LEFT-9-SPLIT", auton4}}, {5, {"RIGHT-9-SPLIT", auton5}},
    {6, {"LEFT-4", auton6}}, {7, {"LEFT-7-SPLIT", auton7}}, {8, {"LEFT-7", auton8}}, 
    {9, {"COUNTER-SAWP", auton9}}, {10, {"RIGHT-4", auton10}}
}; // Maps auton number to name and function

// CHANGE FOR ANY AUTON
// 1 = skills
// 2 = right
// 3 = left

int selectedAuton = std::clamp((int)((autonSelector.get_value() - 1000) / 500), 1, 8);
bool antiJamEnable = true;

// --- Initialization ---
void initialize() {
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    reset.startTracking();
    chassis.calibrate();
    vertical_encoder.reset_position();
    horizontal_encoder.reset_position();
    controller.clear();

    // Background task to update robot info on screen and controller
    Task update_robot_info([&]() {
        int count = 0;
        while (true) {
            if (competition::is_autonomous()) {
                screen::print(E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
                screen::print(E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
                screen::print(E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);
            }

            if (count % 100 == 0) {
                controller.print(0, 0, "Temp: %.1f:%.1f:%d", std::max(left_middle.get_temperature(), right_middle.get_temperature()), std::max(score_motor.get_temperature(), intake_motor.get_temperature()), selectedAuton);
            }
            count++;
            delay(25);
        }
    });
/*
    Task antiJam([&]() {
    while (true) {
        int targetV = intake_motor.get_target_velocity();
        float actualV = intake_motor.get_actual_velocity();

        // 1. Check if we are trying to move fast enough to care about jams (> 100 RPM)
        // 2. Check if the actual speed is less than 66% of the target
        // 3. Ensure the scoring motor is also active (per your original logic)
        if (abs(targetV) > 100 && fabs(actualV) < fabs(targetV / 1.5) && abs(score_motor.get_target_velocity()) > 100 && antiJamEnable) {
            
            // Reversing the intake to clear the jam
            intake_motor.move_velocity(-targetV); 
            pros::delay(200); // Increased slightly to ensure the ring drops back
            
            // Return to original commanded speed
            intake_motor.move_velocity(targetV);
            
            // Settle time: Give the motor a moment to spin back up 
            // before checking for a jam again.
            pros::delay(300); 
        }

        pros::delay(25); // Standard task heartbeat
    }
});
*/

    std::vector<bool> devices_connected = {
        inertial.is_installed(), left_front.is_installed(), left_middle.is_installed(), left_back.is_installed(),
        right_front.is_installed(), right_middle.is_installed(), right_back.is_installed(), intake_motor.is_installed(),
        score_motor.is_installed(), vertical_encoder.is_installed(), horizontal_encoder.is_installed(), score_motor.is_installed(), 
        rightDistance.is_installed(), leftDistance.is_installed(), frontDistance.is_installed(), backDistance.is_installed()
    };
    std::vector<std::string> device_names = {
        "IMU", "L_Front", "L_Middle",
        "L_Back", "R_Front", "R_Middle", "R_Back", "Intake_Motor", "Score_Motor", "V_Tracker", "H_Tracker", "Score_Motor", 
        "Right_Distance", "Left_Distance", "Front_Distance", "Back_Distance"
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
    doublePark.set_value(false); // turn on
    // Select auton using potentiometer before match starts
    while (competition::is_disabled()) {
        // Read the potentiometer value to select auton
        selectedAuton = std::clamp((int)((autonSelector.get_value() - 1000) / 500), 1, 8);
        std::string autonName = autons.at(selectedAuton).first;
        screen::print(E_TEXT_MEDIUM, 3, "                            ");
        screen::print(E_TEXT_MEDIUM, 3, "Auton: %s", autonName.c_str());
        delay(200);
    }
}

void autonomous() {
    vertical_encoder.reset_position();
    horizontal_encoder.reset_position();
    
    // turn to brake if not consistent
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST); //left_motors.set_brake_mode(E_MOTOR_BRAKE_COAST, 1);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST); //right_motors.set_brake_mode(E_MOTOR_BRAKE_COAST, 1);
    doublePark.set_value(false); // turn on
    
    if (autons.count(selectedAuton)) autons.at(9).second();
    else autons.at(1).second();
    
}

void opcontrol() {
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    
    bool intakeToggle, scoreToggle, centerToggle, antenneState, loadToggle, descoreState = false;
    pros::Task* centerTask = nullptr;
    doublePark.set_value(true); // turn off odom
    centerGoal.set_value(false);
    score.set_value(false);
    reset.stopTracking();
    toggle_score(false);
    while (true) {
        // Drive Control
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
        
        // Intake Preroller Toggle
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
            toggle_score(false);
            intakeToggle = !intakeToggle;
            toggle_preroller(intakeToggle);
        }
        
        // Intake Score Control
        if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
            scoreToggle = true;
            score.set_value(scoreToggle);
            toggle_score(scoreToggle);
        } if (controller.get_digital_new_release(E_CONTROLLER_DIGITAL_R1)) {
            scoreToggle = false;
            score.set_value(scoreToggle);
            toggle_score(scoreToggle);
        }

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
            centerToggle = !centerToggle;
            centerGoal.set_value(centerToggle);
            if (centerTask == nullptr && centerToggle)  {
            centerTask = new pros::Task([&](){
                toggle_score(centerToggle, -50, -80);
                pros::delay(250);
                toggle_score(centerToggle, 95, -105);
            });
            } else if (centerTask != nullptr) { centerTask->remove(); delete centerTask; centerTask = nullptr; toggle_score(centerToggle);}
        }

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
            antenneState = !antenneState;
            antenne.set_value(antenneState);
        }

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
            descoreState = !descoreState;
            descore.set_value(descoreState);
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

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
            chassis.tank(20, 30, true);
            toggle_score(false);
            antenne.set_value(true);
            antenneState = true;
            pros::delay(200);
            chassis.tank(-40, -40, true);
            pros::delay(200);
            chassis.tank(65, 65, true);
            pros::delay(200);
            chassis.tank(75, 85, true);
            toggle_preroller(true, 115);
            pros::delay(1500);
            matchLoad.set_value(true);
            intakeToggle = true;
            chassis.tank(60, 70, true);
            pros::delay(250);
            matchLoad.set_value(false);
            loadToggle = false;
            pros::delay(500);
            chassis.tank(0, 0, true);
        }

        // clear bottom
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
            chassis.tank(20, 30, true);
            toggle_score(false);
            antenne.set_value(true);
            antenneState = true;
            pros::delay(200);
            chassis.tank(-40, -40, true);
            pros::delay(200);
            chassis.tank(65, 65, true);
            pros::delay(200);
            chassis.tank(85, 75, true);
            toggle_preroller(true, 115);
            pros::delay(1500);
            matchLoad.set_value(true);
            intakeToggle = true;
            chassis.tank(70, 60, true);
            pros::delay(250);
            matchLoad.set_value(false);
            loadToggle = false;
            pros::delay(500);
            chassis.tank(0, 0, true);
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

/*
void measure_offsets() {
  int iterations = 10;
  double vertical_offset_total = 0.0;
  double horizontal_offset_total = 0.0;

  // Calibrate ONCE at the start
  chassis.calibrate(); 
  pros::delay(2500); 

  for (int i = 0; i < iterations; i++) {
    controller.rumble("..--");
    chassis.cancelAllMotions();
    
    // Reset encoders to 0 before the turn
    vertical_encoder.reset(); 
    horizontal_encoder.reset();
    
    // Use a fixed reference point
    chassis.setPose(0, 0, 0);
    double imu_start = inertial.get_heading();
    
    double target = (i % 2 == 0) ? 90 : 270;

    // slow turn for high accuracy
    chassis.turnTo(target, 5000, {.maxSpeed = 30});
    chassis.waitUntilDone();
    pros::delay(5000);

    // Calculate actual change in angle
    double current_theta = inertial.get_heading();
    double t_delta = degToRad(fabs(current_theta - imu_start));

    // Guard against divide by zero if the robot didn't move
    if (t_delta < 0.01) continue; 

    double v_delta = vertical_tracking_wheel.getDistanceTraveled();
    double h_delta = horizontal_tracking_wheel.getDistanceTraveled();

    vertical_offset_total += (v_delta / t_delta);
    horizontal_offset_total += (h_delta / t_delta);
    
    printf("Iteration %d: V_Off: %f, H_Off: %f\n", i+1, v_delta/t_delta, h_delta/t_delta);
  }

  double final_v_offset = vertical_offset_total / iterations;
  double final_h_offset = horizontal_offset_total / iterations;

  printf("--- FINAL OFFSETS ---\n");
  screen::print(pros::E_TEXT_MEDIUM, 5, "Vertical: %f\n", final_v_offset);
  screen::print(pros::E_TEXT_MEDIUM, 6, "Horizontal: %f\n", final_h_offset);
}
*/