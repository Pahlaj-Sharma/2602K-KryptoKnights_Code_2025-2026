#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <tuple>
#include <vector>

// Remove when PID tuned
pros::Rotation rot_kp(1);
pros::Rotation rot_ki(2);
pros::Rotation rot_kd(3);
pros::adi::DigitalIn limit_switch(4);

void moveLinear(double inches, int timeout, float lead, float maxspeed, float minspeed) {
    // Get position
    const lemlib::Pose currentPose = chassis.getPose(true);

    // Calculate target position based on current pose and distance
    const double targetX = currentPose.x + (inches * std::cos(currentPose.theta));
    const double targetY = currentPose.y + (inches * std::sin(currentPose.theta));

    // Set the chassis to move to the target position
    chassis.moveToPose(targetX, targetY, currentPose.theta, timeout, {.lead = lead, .maxSpeed = maxspeed, .minSpeed = minspeed});
}

void chassisPID(std::string premade, double lat_kp, double lat_ki, double lat_kd, double ang_kp, double ang_ki, double ang_kd){
    // normal, fast, precise
    int selector = 0;

    if (premade=="normal") selector=1; else if(premade=="fast") selector=2; else if(premade=="precise") selector=3;

    switch (selector) {
    case 0:
        // custom
        chassis.lateralPID.kP = lat_kp; chassis.lateralPID.kI = lat_ki; chassis.lateralPID.kD = lat_kd;
        chassis.angularPID.kP = ang_kp; chassis.angularPID.kI = ang_ki; chassis.angularPID.kD = ang_kd;
        break;
    case 1:
        //normal
        chassis.lateralPID.kP = LATERAL_KP; chassis.lateralPID.kI = LATERAL_KI; chassis.lateralPID.kD = LATERAL_KD; 
        chassis.angularPID.kP = ANGULAR_KP; chassis.angularPID.kI = ANGULAR_KI; chassis.angularPID.kD = ANGULAR_KD;
        break;
    case 2:
        //fast
        chassis.lateralPID.kP = F_LATERAL_KP; chassis.lateralPID.kI = F_LATERAL_KI; chassis.lateralPID.kD = F_LATERAL_KD;
        chassis.angularPID.kP = F_ANGULAR_KP; chassis.angularPID.kI = F_ANGULAR_KI; chassis.angularPID.kD = F_ANGULAR_KD;
        break;
    case 3:
        //precise
        chassis.lateralPID.kP = P_LATERAL_KP; chassis.lateralPID.kI = P_LATERAL_KI; chassis.lateralPID.kD = P_LATERAL_KD;
        chassis.angularPID.kP = P_ANGULAR_KP; chassis.angularPID.kI = P_ANGULAR_KI; chassis.angularPID.kD = P_ANGULAR_KD;
        break;
        }
}

void resetOdometry(int threshold) {
    const double MM_IN = 0.03937; // Conversion factor from mm to inches
    const int FIELD_SIZE = 70; // Size of the field in inches
    // Get the chassis pose
    const lemlib::Pose pose = chassis.getPose(true);

    // Get sensor values
    std::vector<std::tuple<double, double, std::string>> sensor_values = {
        {frontDistance.get() * MM_IN, DS_FRONT_CENTER, "Y"}, {backDistance.get() * MM_IN, DS_BACK_CENTER, "Y"},
        {leftDistance.get() * MM_IN, DS_LEFT_CENTER, "X"}, {rightDistance.get() * MM_IN, DS_RIGHT_CENTER, "X"}};
    // Sort the sensor values
    std::sort(sensor_values.begin(), sensor_values.end());
    // Check if both sensors calculate same axis and if the first sensor is not too far away
    if (std::get<2>(sensor_values[0]) == std::get<2>(sensor_values[1]) || std::get<0>(sensor_values[0]) > 10) return;
    // Calculate the maximum theta for the pose
    const double max_theta = std::max(std::abs(std::cos(pose.theta)), std::abs(std::sin(pose.theta)));

    // Find total distances
    const double distance1 = (std::get<0>(sensor_values[0]) * max_theta) + (std::get<1>(sensor_values[0]) * max_theta);
    const double distance2 = (std::get<0>(sensor_values[1]) * max_theta) + (std::get<1>(sensor_values[1]) * max_theta);
    double calculated_x, calculated_y;

    // Check which distance is applied to which axis based on the angle
    if ((0 <= pose.theta && pose.theta < M_PI_4) || (M_7PI_4 < pose.theta && pose.theta <= M_2_PI) || (M_3PI_4 < pose.theta && pose.theta < M_5PI_4)){
        if (std::get<2>(sensor_values[0]) == "X"){
        calculated_x = distance1; calculated_y = distance2;} else {calculated_x = distance2; calculated_y = distance1;}
    } else { // Other angles
        if (std::get<2>(sensor_values[0]) == "X"){
        calculated_x = distance2; calculated_y = distance1;} else {calculated_x = distance1; calculated_y = distance2;}
    }

    // Check the quadrant
    if (pose.x > 0) calculated_x = FIELD_SIZE - calculated_x; else calculated_x -= FIELD_SIZE;
    if (pose.y > 0) calculated_y = FIELD_SIZE - calculated_y; else calculated_y -= FIELD_SIZE;

    // Set the pose
    if ((std::abs(calculated_x - pose.x) < threshold) && (std::abs(calculated_y - pose.y) < threshold)){
        chassis.setPose(calculated_x, calculated_y, chassis.getPose().theta);
    } else if (std::abs(calculated_x - pose.x) < threshold){
        chassis.setPose(calculated_x, pose.y, chassis.getPose().theta);
    } else if (std::abs(calculated_y - pose.y) < threshold){
        chassis.setPose(pose.x, calculated_y, chassis.getPose().theta);
    } else return;
}

void tunePID(){
    // Comment when PID tuned
    double initial_kp = chassis.lateralPID.kP; double initial_ki = chassis.lateralPID.kI; double initial_kd = chassis.lateralPID.kD;
	const int initial_rot_kp_pos = rot_kp.get_position();
    const int initial_rot_ki_pos = rot_ki.get_position();
    const int initial_rot_kd_pos = rot_kd.get_position();
	#define KP_SCALE_FACTOR 0.1f // Adjust Kp by 0.1 for every degree of rotation
	#define KI_SCALE_FACTOR 0.01f // Adjust Ki
	#define KD_SCALE_FACTOR 0.1f // Adjust Kd
	while (true) {
		double delta_kp = (rot_kp.get_position() - initial_rot_kp_pos) * KP_SCALE_FACTOR;
        double delta_ki = (rot_ki.get_position() - initial_rot_ki_pos) * KI_SCALE_FACTOR;
        double delta_kd = (rot_kd.get_position() - initial_rot_kd_pos) * KD_SCALE_FACTOR;
		chassis.lateralPID.kP = initial_kp + delta_kp; // Interchange between lateral and angular
		chassis.lateralPID.kI = initial_ki + delta_ki;
		chassis.lateralPID.kD = initial_kd + delta_kd;
		controller.print(0, 0, "kP: %f", chassis.lateralPID.kP);
        controller.print(1, 0, "kI: %f", chassis.lateralPID.kI);
        controller.print(2, 0, "kD: %f", chassis.lateralPID.kD);
		if (limit_switch.get_new_press()){
			controller.rumble("-");
			chassis.calibrate();
			while (inertial.is_calibrating()) {pros::delay(10);} inertial.reset();
			chassis.setPose(0, 0, 0);
			pros::delay(100);
			chassis.moveToPoint(0, 24, 10000); // Change based on movement
			chassis.waitUntilDone();
			controller.rumble(".");
			controller.clear();
		}
		pros::delay(100);
	}
}