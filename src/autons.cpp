#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "pros/distance.hpp"
#include "robot_config.hpp"
#include <cmath>
#include <tuple>
#include <vector>

void auton1() {
    chassis.setPose(0, 0, 0);
    moveLinear(12);
    chassisPID("precise");
    resetOdometry();
}
void auton2() {
    
}
void auton3() {
    
}
void auton4() {
    
}
void auton5() {
    
}
void auton6() {
    
}
void auton7() {
    
}
void auton8() {
    
}
void auton9() {
    
}
void auton10() {
    
}


void moveLinear(double inches, int timeout, float maxspeed, float minspeed) {
    // Get position
    lemlib::Pose currentPose = chassis.getPose(true);

    // Calculate target position based on current pose and distance
    double targetX = currentPose.x + (inches * std::cos(currentPose.theta));
    double targetY = currentPose.y + (inches * std::sin(currentPose.theta));

    // Set the chassis to move to the target position
    chassis.moveToPose(targetX, targetY, currentPose.theta, timeout, {.lead = 0.2, .maxSpeed = maxspeed, .minSpeed = minspeed});
}

void chassisPID(std::string premade, double lat_kp, double lat_ki, double lat_kd, double lat_slew, double ang_kp, double ang_ki, double ang_kd){
    // normal, fast, precise
    int selector = 0;

    if (premade=="normal") selector=1; else if(premade=="fast") selector=2; else if(premade=="precise") selector=3;

    switch (selector) {
    case 0:
        chassis.lateralPID.kP = lat_kp; chassis.lateralPID.kI = lat_ki; chassis.lateralPID.kD = lat_kd;
        chassis.angularPID.kP = ang_kp; chassis.angularPID.kI = ang_ki; chassis.angularPID.kD = ang_kd;
    case 1:
        //normal
        chassis.lateralPID.kP = LATERAL_KP; chassis.lateralPID.kI = LATERAL_KI; chassis.lateralPID.kD = LATERAL_KD; 
        chassis.angularPID.kP = ANGULAR_KP; chassis.angularPID.kI = ANGULAR_KI; chassis.angularPID.kD = ANGULAR_KD;
    case 2:
        //fast
        chassis.lateralPID.kP = F_LATERAL_KP; chassis.lateralPID.kI = F_LATERAL_KI; chassis.lateralPID.kD = F_LATERAL_KD;
        chassis.angularPID.kP = F_ANGULAR_KP; chassis.angularPID.kI = F_ANGULAR_KI; chassis.angularPID.kD = F_ANGULAR_KD;
    case 3:
        //precise
        chassis.lateralPID.kP = P_LATERAL_KP; chassis.lateralPID.kI = P_LATERAL_KI; chassis.lateralPID.kD = P_LATERAL_KD;
        chassis.angularPID.kP = P_ANGULAR_KP; chassis.angularPID.kI = P_ANGULAR_KI; chassis.angularPID.kD = P_ANGULAR_KD;
        }
}

void resetOdometry(int threshold) {
    const double MM_IN = 0.03937; // Conversion factor from mm to inches
    const int FIELD_SIZE = 70; // Size of the field in inches
    // Get the chassis pose
    lemlib::Pose pose = chassis.getPose(true);

    // Get sensor values
    std::vector<std::tuple<double, double, std::string>> sensor_values = {
        {frontDistance.get() * MM_IN, DS_FRONT_CENTER, "Y"}, {backDistance.get() * MM_IN, DS_BACK_CENTER, "Y"},
        {leftDistance.get() * MM_IN, DS_LEFT_CENTER, "X"}, {rightDistance.get() * MM_IN, DS_RIGHT_CENTER, "X"}};
    // Sort the sensor values
    std::sort(sensor_values.begin(), sensor_values.end());
    // Check if both sensors calculate same axis and if the first sensor is not too far away
    if (std::get<2>(sensor_values[0]) == std::get<2>(sensor_values[1]) || std::get<0>(sensor_values[0]) > 10) return;
    // Calculate the maximum theta for the pose
    double max_theta = std::max(std::abs(std::cos(pose.theta)), std::abs(std::sin(pose.theta)));

    // Calculate distances to the sensor
    double dist_to_sensor1 = std::get<0>(sensor_values[0]) * max_theta; // First
    double dist_to_sensor2 = std::get<0>(sensor_values[1]) * max_theta; // Second
    // Calculate distance to the center of the robot
    double dist_to_center1 = std::get<1>(sensor_values[0]) * max_theta; // First
    double dist_to_center2 = std::get<1>(sensor_values[1]) * max_theta; // Second
    // Find total distances
    double distance1 = dist_to_sensor1 + dist_to_center1;
    double distance2 = dist_to_sensor2 + dist_to_center2;
    double calculated_x, calculated_y;

    // Check which distance is applied to which axis based on the angle
    if (0 <= pose.theta < M_PI_4 || M_7PI_4 < pose.theta <= M_2_PI || M_3PI_4 < pose.theta < M_5PI_4){ // 315 - 90 degrees
        if (std::get<2>(sensor_values[0]) == "X"){
        calculated_x = distance1; calculated_y = distance2;} else {calculated_x = distance2; calculated_y = distance1;}
    } else { // Other angles
        if (std::get<2>(sensor_values[0]) == "X"){
        calculated_x = distance2; calculated_y = distance1;} else {calculated_x = distance1; calculated_y = distance2;}}

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
