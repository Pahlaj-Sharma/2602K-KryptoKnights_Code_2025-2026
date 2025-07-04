#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <ctime>
#include <iostream>
#include <ostream>
#include <map>
#include <string>
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Define Ports
pros::MotorGroup right_motors({11, 13, 12}, pros::MotorGearset::blue); // Right Motors
pros::MotorGroup left_motors({-14, -15, -16}, pros::MotorGearset::blue); // Left Motors
// Inertial
pros::Imu imu(2);
// Horizontal Odometry Tracker
pros::Rotation horizontal_encoder(-3);
// Vertical Odometry Tracker
pros::Rotation vertical_encoder(17);
// Auton Selector
pros::adi::Potentiometer autonSelector(6);
// Team Selector
pros::adi::DigitalIn teamSelector(7);
// Distance Sensors
pros::Distance rightDistance(3);
pros::Distance leftDistance(3);
pros::Distance frontDistance(3);
pros::Distance backDistance(3);
// Define Drivetrain
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.875, lemlib::Omniwheel::NEW_275, 450, 2);
//Define Odom Sensors
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.25);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0.0625);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);
// Define PID Controllers
lemlib::ControllerSettings lateral_controller(7, 0, 9, 3, 1, 100, 3, 500, 15);
lemlib::ControllerSettings angular_controller(2, 0, 16, 3, 1, 100, 3, 500, 0);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
//Global Variables
int team = 0;
// Toggle Variables
int autonSelect = 1;


/** * Runs initialization code. This occurs as soon as the program is started. *
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds. */
void initialize() {
	left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); 
	right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); 
	horizontal_encoder.reset_position(); 
	vertical_encoder.reset_position();
	pros::lcd::initialize();
	chassis.calibrate();
	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x); // x
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y); // y
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}
/**
* Runs while the robot is in the disabled state of Field Management System or * the VEX Competition Switch, following either autonomous or opcontrol. When * the robot is enabled, this task will exit.
*/
void disabled() {}

 /**
* Runs after initialize(), and before autonomous when connected to the Field
* Management System or the VEX Competition Switch. This is intended for
* competition-specific initialization routines, such as an autonomous selector
* on the LCD. *
* This task will exit when the robot is enabled and autonomous or opcontrol
* starts. */
void competition_initialize() {
	/*
	std::string teamtype = "RED";
	std::map<int, std::string> auton_map = {
		{1, "SKILLS, POSITION FACING TOWARDS ALLIANCE STAKE"},
		{2, "NEGATIVE SIDE AWP GETS ALLIANCE STAKE, POSITION FACING TOWARDS THE ALLIANCE STAKE"},
		{3, "POSITIVE SIDE AWP GETS MIDDLE RINGS, POSITION FACING TOWARDS GOAL"},
		{4, "NEGATIVE SIDE ELIMS GETS ALLIANCE STAKE, POSITION FACING TOWARDS GOAL"},
		{5, "POSITIVE SIDE ELIMS GETS MIDDLE RINGS, POSITION FACING TOWARDS GOAL"},
		{6, "SOLO AWP GETS AWP SOLO, POSITION ALLIANCE STAKE ON NEG SIDE"},
		{7, "Auton7"},
		{8, "Auton8"},
		{9, "Auton9"},
		{10, "Auton10"},
	};
	while (true) {
		if (teamSelector.get_new_press()) {
			team = (team + 1) % 2;
			if (team == 1){
				teamtype = "RED";
			} else {
				teamtype = "BLUE";
			}
		}
		pros::screen::print(pros::E_TEXT_MEDIUM, 1, auton_map[autonSelect].c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM, 3, teamtype.c_str());
		pros::delay(150);
		pros::screen::erase();
	}
	*/
}
//Auton Paths
namespace autons { 
	void auton1(){ // Skills
		pros::delay(100);
		chassis.setPose(0, 0, 0);
		chassis.moveToPose(45, 24, 90, 4500, {.lead = 0.7, .minSpeed = 60});
		
		}
	
}
/**
* Runs the user autonomous code. This function will be started in its own task * with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the
autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes. *
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off. */
void autonomous() {
	// 0 = blue, 1 = red
	if (autonSelect == 1) {
		autons::auton1();}
}
/**
* Runs the operator control code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the operator
* control mode. *
* If no competition control is connected, this function will run immediately
* following initialize(). *
* If the robot is disabled or communications is lost, the
* operator control task will be stopped. Re-enabling the robot will restart the
* task, not resume it from where it left off. */
void opcontrol() {
	while (true) {
	// Driving
	int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); 
	int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 
	chassis.arcade(leftY, rightX);
	//Delay for resources
	pros::delay(25);
	} 
}
