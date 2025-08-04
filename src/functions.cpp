#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "functions.hpp"
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

void chassisPID(PIDPreset premade) {
    PIDConstants lateral_pid;
    PIDConstants angular_pid;
    switch (premade) {
        case PIDPreset::normal:
            lateral_pid = LATERAL_PID;
            angular_pid = ANGULAR_PID;
            break;
        case PIDPreset::fast:
            // Define your 'fast' PID constants here
            // Example:
            lateral_pid = F_LATERAL_PID;
            angular_pid = F_ANGULAR_PID;
            break;
        case PIDPreset::precise:
            // Define your 'precise' PID constants here
            // Example:
            lateral_pid = P_LATERAL_PID;
            angular_pid = P_ANGULAR_PID;
            break;
        default:
            // Default to normal PID
            lateral_pid = LATERAL_PID;
            angular_pid = ANGULAR_PID;
            break;
    chassis.lateralPID.kP = lateral_pid.kP; chassis.lateralPID.kI = lateral_pid.kI; chassis.lateralPID.kD = lateral_pid.kD;
    chassis.angularPID.kP = angular_pid.kP; chassis.angularPID.kI = angular_pid.kI; chassis.angularPID.kD = angular_pid.kD;
    }}

void chassisPID(double lat_kp, double lat_ki, double lat_kd, double ang_kp, double ang_ki, double ang_kd) {
    // Set the lateral PID constants
    chassis.lateralPID.kP = lat_kp; chassis.lateralPID.kI = lat_ki; chassis.lateralPID.kD = lat_kd;
    // Set the angular PID constants
    chassis.angularPID.kP = ang_kp; chassis.angularPID.kI = ang_ki; chassis.angularPID.kD = ang_kd;
}

struct SensorData {
    double distance;
    double offset;
    char axis; // 'X' or 'Y'
    // Optional: add a comparison operator to sort the struct directly
    bool operator<(const SensorData& other) const {
        return this->distance < other.distance;
    }};

void resetOdometry(double threshold) {
    constexpr double MM_IN = 0.03937;
    constexpr double FIELD_SIZE = 70.0; // Using double for consistency with other calculations

    // Store the pose once to avoid multiple calls and ensure consistency
    const lemlib::Pose initial_pose = chassis.getPose(true);

    // Populate a vector of structs for better readability than a tuple
    std::vector<SensorData> sensor_readings = {
        {frontDistance.get() * MM_IN, DS_FRONT_CENTER, 'Y'},
        {backDistance.get() * MM_IN, DS_BACK_CENTER, 'Y'},
        {leftDistance.get() * MM_IN, DS_LEFT_CENTER, 'X'},
        {rightDistance.get() * MM_IN, DS_RIGHT_CENTER, 'X'}
    };
    // Sort the sensor readings based on distance
    std::sort(sensor_readings.begin(), sensor_readings.end());

    // Check if the two closest sensors are on the same axis or if the first sensor is too far
    if (sensor_readings[0].axis == sensor_readings[1].axis || sensor_readings[0].distance > 10.0) return;

    // Get the two closest sensors
    const SensorData& sensor1 = sensor_readings[0];
    const SensorData& sensor2 = sensor_readings[1];

    // Calculate max_theta
    const double max_theta = std::max(std::abs(std::cos(initial_pose.theta)), std::abs(std::sin(initial_pose.theta)));

    // Calculate total distances
    const double distance1 = (sensor1.distance * max_theta) + (sensor1.offset * max_theta);
    const double distance2 = (sensor2.distance * max_theta) + (sensor2.offset * max_theta);

    double calculated_x, calculated_y;
    // Use an intuitive variable name for the condition
    const bool standard_angle_axis = (0 <= initial_pose.theta && initial_pose.theta < M_PI_4) || 
                                       (M_7PI_4 < initial_pose.theta && initial_pose.theta <= M_2_PI) ||
                                       (M_3PI_4 < initial_pose.theta && initial_pose.theta < M_5PI_4);

    if (standard_angle_axis) {
        if (sensor1.axis == 'X') {
            calculated_x = distance1; calculated_y = distance2;
        } else {
            calculated_x = distance2; calculated_y = distance1;
        }
    } else { // Other angles
        if (sensor1.axis == 'X') {
            calculated_x = distance2; calculated_y = distance1;
        } else {
            calculated_x = distance1; calculated_y = distance2;
        }
    }

    // Simplify quadrant checks for calculated_x and calculated_y
    if (initial_pose.x > 0) calculated_x = FIELD_SIZE - calculated_x;
    if (initial_pose.x < 0) calculated_x -= FIELD_SIZE;

    if (initial_pose.y > 0) calculated_y = FIELD_SIZE - calculated_y;
    if (initial_pose.y < 0) calculated_y -= FIELD_SIZE;

    // Set the pose if within threshold
    const double x_threshold = std::abs(calculated_x - initial_pose.x);
    const double y_threshold = std::abs(calculated_y - initial_pose.y);
    const lemlib::Pose current_pose = chassis.getPose();

    if ((x_threshold < threshold) && (y_threshold < threshold)) {
        chassis.setPose(calculated_x, calculated_y, current_pose.theta);
    } else if (x_threshold < threshold) {
        chassis.setPose(calculated_x, current_pose.y, current_pose.theta);
    } else if (y_threshold < threshold) {
        chassis.setPose(current_pose.x, calculated_y, current_pose.theta);
    }
}

void tunePID(){
    // Comment when PID tuned
    double initial_kp = chassis.lateralPID.kP; double initial_ki = chassis.lateralPID.kI; double initial_kd = chassis.lateralPID.kD;
	const int initial_rot_kp_pos = rot_kp.get_position();
    const int initial_rot_ki_pos = rot_ki.get_position();
    const int initial_rot_kd_pos = rot_kd.get_position();
	constexpr float KP_SCALE_FACTOR = 0.1f; // Adjust Kp by 0.1 for every degree of rotation
	constexpr float KI_SCALE_FACTOR = 0.01f; // Adjust Ki
	constexpr float KD_SCALE_FACTOR = 0.1f; // Adjust Kd
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