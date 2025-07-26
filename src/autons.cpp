#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include <cmath>

ASSET(path_jerryio_txt);

void auton1() {
    chassis.setPose(0, 0, 0);
    moveLinear(12);
    chassisPID("precise");
    chassis.follow(path_jerryio_txt, 3, 20000);
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
        lemlib::Pose currentPose = chassis.getPose();
        double currentThetaRad = lemlib::degToRad(currentPose.theta);
        double targetX = currentPose.x + (inches * std::cos(currentThetaRad));
        double targetY = currentPose.y + (inches * std::sin(currentThetaRad));
        chassis.moveToPose(targetX, targetY, currentPose.theta, timeout, {.lead = 0.2, .maxSpeed = maxspeed, .minSpeed = minspeed});
    }

void chassisPID(std::string premade, double lat_kp, double lat_ki, double lat_kd, double lat_slew, double ang_kp, double ang_ki, double ang_kd){
    // normal, fast, precise
    int selector = 0;
    if (premade=="normal"){selector=1;}else if(premade=="fast"){selector=2;}else if(premade=="precise"){selector=3;}
    switch (selector) {
    case 0:
        chassis.lateralPID.kP = lat_kp;
        chassis.lateralPID.kI = lat_ki;
        chassis.lateralPID.kD = lat_kd;
        chassis.angularPID.kP = ang_kp;
        chassis.angularPID.kI = ang_ki;
        chassis.angularPID.kD = ang_kd;
    case 1:
        //normal
        chassis.lateralPID.kP = LATERAL_KP;
        chassis.lateralPID.kI = LATERAL_KI;
        chassis.lateralPID.kD = LATERAL_KD;
        chassis.angularPID.kP = ANGULAR_KP;
        chassis.angularPID.kI = ANGULAR_KI;
        chassis.angularPID.kD = ANGULAR_KD;
    case 2:
        //fast
        chassis.lateralPID.kP = F_LATERAL_KP;
        chassis.lateralPID.kI = F_LATERAL_KI;
        chassis.lateralPID.kD = F_LATERAL_KD;
        chassis.angularPID.kP = F_ANGULAR_KP;
        chassis.angularPID.kI = F_ANGULAR_KI;
        chassis.angularPID.kD = F_ANGULAR_KD;
    case 3:
        //precise
        chassis.lateralPID.kP = P_LATERAL_KP;
        chassis.lateralPID.kI = P_LATERAL_KI;
        chassis.lateralPID.kD = P_LATERAL_KD;
        chassis.angularPID.kP = P_ANGULAR_KP;
        chassis.angularPID.kI = P_ANGULAR_KI;
        chassis.angularPID.kD = P_ANGULAR_KD;
        }
    }
void resetOdometry(pros::Distance sensor1, std::string axis, double dist1_center) {
    // TODO - Make this work with all sensors
    // resets odometry pos using dist sensors (all calculations here are in mm, then converted to inches)
    lemlib::Pose pose = chassis.getPose(true);
    // Calculate the horizontal offset caused by the angle over the vertical height
    double sensor_val = sensor1.get() * 0.0394; // Convert mm to inches
    double max_theta = std::max(std::abs(std::cos(pose.theta)), std::abs(std::sin(pose.theta)));
    // Calculate distance to the sensor
    double dist_to_sensor = sensor_val * max_theta;
    // Calculate the distance to the center of robot
    double dist_to_center = dist1_center * max_theta;
    double distance = dist_to_sensor + dist_to_center;
    double calculated_x; double calculated_y;
    // Check the quadrant
    if ((pose.x > 0 && pose.y > 0) || (pose.x > 0 && pose.y < 0)) { // Q1 or Q4 (Right Half)
        calculated_x = 144 - distance;
        calculated_y = (pose.y > 0) ? calculated_x : -calculated_x; // If Y positive, y=x, else y=-x
    } else { // pose.x < 0 (Q2 or Q3 - Left Half)
        calculated_x = -144 + distance;
        calculated_y = (pose.y > 0) ? -calculated_x : calculated_x; // If Y positive, y=-x, else y=x
    }
    // Check which axis and within 3 inches of the original position + dist1_center
    if (axis == "X" && (std::abs(calculated_x - pose.x) < dist1_center + 3)){
        chassis.setPose(calculated_x, pose.y, pose.theta);
    } else if (axis == "Y" && (std::abs(calculated_y - pose.y) < dist1_center + 3)){
        chassis.setPose(pose.x, calculated_y, pose.theta);
    }
    }