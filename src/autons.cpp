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

void resetOdometry(pros::Distance sensor1, std::string axis, int dist_center){
    // resets odometry pos using dist sensors (all calculations here are in mm, then converted to inches)
    double chassis_theta = chassis.getPose(true).theta;
    // Calculate the horizontal offset caused by the angle over the vertical height
    double dist_to_sens = sensor1.get() * std::cos(chassis_theta);
    // Calculate distance by subtracting the offset from the total horizontal distance
    double dist_to_cent = std::cos(chassis_theta) * dist_center;
    double distance_in = (dist_to_cent + dist_to_sens) * 0.0394;
    double calculated_x = 0;
    double calculated_y = 0;
    if (chassis.getPose().x > 0 && chassis.getPose().y > 0){
        calculated_x = 144 - distance_in;
        calculated_y = 144 - distance_in;
    } else if (chassis.getPose().x < 0 && chassis.getPose().y > 0) {
        calculated_x = -144 + distance_in;
        calculated_y = 144 - distance_in;
    } else if (chassis.getPose().x < 0 && chassis.getPose().y < 0) {
        calculated_x = -144 + distance_in;
        calculated_y = -144 + distance_in;
    } else if (chassis.getPose().x > 0 && chassis.getPose().y < 0) {
        calculated_x = 144 - distance_in;
        calculated_y = -144 + distance_in;
    }
    if (axis == "X"){
        chassis.setPose(calculated_x, chassis.getPose().y, chassis.getPose().theta);
    } else {
        chassis.setPose(chassis.getPose().x, calculated_y, chassis.getPose().theta);
    }
    }