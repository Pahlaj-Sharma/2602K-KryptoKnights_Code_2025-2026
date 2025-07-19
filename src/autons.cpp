#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "main.h"
#include "autons.hpp"
#include "robot_config.hpp"
#include "lemlib/api.hpp"
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

/* void resetOdometry() {
    int front = frontDistance.get()-DS_FRONT_INDENT_INCHES; std::vector<int> frontData = {(front < 200) ? front : 1000, frontDistance.get_confidence(), (front < 200) ? 1 : 0};
    int back = backDistance.get()-DS_BACK_INDENT_INCHES; std::vector<int> backData = {(back < 200) ? back : 1000, backDistance.get_confidence(), (back < 200) ? 1 : 0};
    int left = leftDistance.get()-DS_LEFT_INDENT_INCHES; std::vector<int> leftData = {(left < 200) ? left : 1000, leftDistance.get_confidence(), (left < 200) ? 1 : 0};
    int right = rightDistance.get()-DS_RIGHT_INDENT_INCHES; std::vector<int> rightData = {(right < 200) ? right : 1000, rightDistance.get_confidence(), (right < 200) ? 1 : 0};
    double side_a = frontData[0];int distance_id = 1;
    if (backData[0] < side_a){side_a=backData[0];distance_id=2;}
    if (leftData[0] < side_a){side_a=leftData[0];distance_id=3;}
    if (rightData[0] < side_a){side_a=rightData[0];distance_id=4;}
    double side_b = 0.0;
    double ds_center = 0.0;
    switch (distance_id) {
    case 1: side_b = FRONT_CENTER; ds_center = DS_FRONT_CENTER_INCHES;
    case 2: side_b = BACK_CENTER; ds_center = DS_BACK_CENTER_INCHES;
    case 3: side_b = LEFT_CENTER; ds_center = DS_LEFT_CENTER_INCHES;
    case 4: side_b = RIGHT_CENTER; ds_center = DS_RIGHT_CENTER_INCHES;}
    double angle_C = std::fabs(lemlib::degToRad(lemlib::angleError(360, chassis.getPose().theta)));
    double distance_value = std::sqrt(side_a * side_a + side_b * side_b - 2 * side_a * side_b * std::cos(angle_C));
    distance_value *= 0.039;
    double calculated_x = 0.0; double calculated_y = 0.0;
    double cos_value = std::cos(chassis.getPose(true).theta) * distance_value + ds_center;
    double sin_value = std::sin(chassis.getPose(true).theta) * distance_value + ds_center;
    if (chassis.getPose().x > 0 && chassis.getPose().y > 0) { // First
        calculated_x = 144 - cos_value;
        calculated_y = 144 - sin_value;
    } else if (chassis.getPose().x < 0 && chassis.getPose().y > 0) { // Second
        calculated_x = -144 + cos_value;
        calculated_y = 144 - sin_value;
    } else if (chassis.getPose().x < 0 && chassis.getPose().y < 0) { // Third
        calculated_x = -144 + cos_value;
        calculated_y = -144 + sin_value;
    } else if (chassis.getPose().x > 0 && chassis.getPose().y < 0) { // Fourth
        calculated_x = 144 - cos_value;
        calculated_y = -144 + sin_value;
    }
    if ((std::fabs(chassis.getPose().x) - std::fabs(calculated_x) > 3) || (std::fabs(chassis.getPose().y) - std::fabs(calculated_y) > 3)) {
        return;
    } else {
        chassis.setPose(calculated_x, calculated_y, chassis.getPose().theta);
    }

} */
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