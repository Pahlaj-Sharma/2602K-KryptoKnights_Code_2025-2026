#include "lemlib/asset.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"

ASSET(path_txt);
ASSET(path2_txt);

void auton1() {
    chassis.setPose(0, 0, 0);
    chassis.setPID(lemlib::Chassis::PIDPreset::normal);
    chassis.setPID(2.0, 0.0, 16.0, 2.0, 0.0, 16.0); // Custom PID values for this auton
    chassis.moveLinear(12);
    chassis.resetOdometry();
    chassis.follow(path_txt, 2, 10000);
    chassis.moveToPose(14, 14, 90, 3000, {}, {{2, 0, 16}}, {{2, 0, 16}});
}

void auton2() {
    chassis.setPose(0, 0, 0);
    chassis.setPID(lemlib::Chassis::PIDPreset::normal);
    chassis.moveToPoint(7.778, -26.45, 5000);
    chassis.moveToPoint(31.927, -2.549, 5000);
    chassis.moveToPoint(35.662, 22.596, 5000);
    chassis.moveToPoint(17.487, 55.459, 5000);

}

void auton3() {
    chassis.setPose(0, 0, 0);
    chassis.setPID(lemlib::Chassis::PIDPreset::normal);
    chassis.setPID(2.0, 0.0, 16.0, 2.0, 0.0, 16.0); // Custom PID values for this auton
    chassis.moveLinear(12);
    chassis.resetOdometry();
    chassis.ramsete(path2_txt, 2, 0.7, 10000);
    chassis.stanley(path_txt, 7, 1, 0.5, 10000);
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
