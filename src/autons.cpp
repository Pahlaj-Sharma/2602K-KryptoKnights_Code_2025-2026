#include "main.h"
#include "pahlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"

using namespace pahlib;

void auton1() {
    // SKILLS
    chassis.setPose(-49.5, -15.5, 180);
    chassis.moveTo(-49.5, -47, 180, 1500, {.maxSpeed = 70});
    chassis.turnTo(270, 500);
    matchLoad.set_value(true);
    pros::delay(100);
    chassis.moveTo(-59, -48, 270, 1000, {.maxSpeed = 60});
    toggle_preroller(true);
    pros::delay(1500);
    chassis.moveTo(-55, -48, 270, 1000, {.maxSpeed = 60});
    chassis.moveTo(-60, -48, 270, 1000, {.maxSpeed = 60});
    pros::delay(400);
    chassis.moveTo(-23, -47, 270, 800, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    toggle_score(true);
    pros::delay(3000);
    toggle_score(false);
    chassis.moveTo(-40, -48, 270, 1000);
    chassis.turnTo(0, 500);
    toggle_pto(true);
    chassis.moveTo(-40, 48, 0, 2500, {.maxSpeed = 80});
    chassis.waitUntilDone();
    toggle_pto(false);
    chassis.turnTo(270, 500);
    chassis.moveTo(-23, 47, 270, 800, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.setPose(-29.25, 48, chassis.getPose().theta);
    chassis.moveTo(-59, 48, 270, 1500, {.maxSpeed = 60});
    matchLoad.set_value(true);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.moveTo(-55, 48, 270, 1000, {.maxSpeed = 60});
    chassis.moveTo(-59, 48, 270, 1000, {.maxSpeed = 60});
    pros::delay(200);
    chassis.moveTo(-23, 48, 270, 800, {.forwards = false, .minSpeed = 60});
    chassis.waitUntil(7);
    toggle_score(true);
    chassis.waitUntilDone();
    pros::delay(2500);
    matchLoad.set_value(false);
    chassis.moveTo(-64, 20, 180, 1500, {.maxSpeed = 70});
    chassis.turnTo(15, 1000);
    toggle_pto(true);
    chassis.moveTo(-63, 0, 3000, {.forwards = false, .minSpeed = 127});

}

void auton2() {
    // RIGHT
    chassis.setPose(-47.75, -13.75, 90);
    toggle_pto(true);
    toggle_preroller(true);
    chassis.turnTo(-15, -24, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-16, -25, 115, 1000, {.minSpeed = 30});
    chassis.moveTo(-9.5, -44, 1000, {.maxSpeed = 60});
    chassis.waitUntil(3);
    toggle_preroller(true, 40);
    chassis.waitUntil(10);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveTo(-24, -33, 1000, {.forwards = false});
    toggle_preroller(false);
    chassis.swingTo(270, DriveSide::RIGHT, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 60});
    chassis.moveTo(-20, -47.75, 270, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.moveTo(-15, -47.75, 400, {.forwards = false});
    chassis.swingTo(270, DriveSide::RIGHT, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.waitUntilDone();
    toggle_score(true);
    pros::delay(1500);
    chassis.setPose(-29.25, -48, chassis.getPose().theta);
    toggle_score(false);
    chassis.moveTo(-65, -48, 270, 1000, {.maxSpeed = 60});
    matchLoad.set_value(true);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.moveTo(-23, -49, 270, 800, {.forwards = false, .minSpeed = 60});
    chassis.waitUntil(13);
    toggle_score(true);
    chassis.waitUntilDone();
    matchLoad.set_value(false);
    chassis.moveTo(-30, -48, 1000);
    chassis.moveTo(-23, -49, 800, {.forwards = false, .minSpeed = 60});
}

void auton3() {
    // LEFT
    chassis.setPose(-47.75, 13.75, 90);
    toggle_pto(true);
    toggle_preroller(true);
    chassis.turnTo(-15, 24, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-16, 25, 65, 1000, {.minSpeed = 30});
    chassis.moveTo(-9, 44, 1000, {.maxSpeed = 60});
    chassis.waitUntil(3);
    toggle_preroller(true, 40);
    chassis.waitUntil(10);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveTo(-20, 21, 1000, {.forwards = false});
    toggle_preroller(false);
    chassis.turnTo(-25, 26, 500);
    chassis.moveTo(-14, 13, 315, 1000, {.forwards = false});
    chassis.waitUntil(8);
    toggle_score(true, 100, 35);
    pros::delay(200);
    toggle_score(false);
    chassis.moveTo(-47, 47, 1500, {.maxSpeed = 90});
    chassis.turnTo(-68, 48, 500);
    chassis.moveTo(-23, 50, 270, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    toggle_score(true);
    pros::delay(1000);   
    chassis.setPose(-29.25, 48, chassis.getPose().theta);
    toggle_score(false);
    chassis.moveTo(-65, 48, 270, 1500, {.maxSpeed = 60});
    matchLoad.set_value(true);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveTo(-23, 48, 270, 800, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    toggle_score(true);
    matchLoad.set_value(false);
}

void auton4() {
// NEW SKILLS
/*
    chassis.setPose(-49.5, -15.5, 180);
    antenne.set_value(true);
    chassis.moveTo(-49.5, -45.5, 180, 1500, {.minSpeed = 40});
    chassis.turnTo(270, 500);
    matchLoad.set_value(true);
    pros::delay(100);
    chassis.moveTo(-63.5, -46, 270, 1000, {.maxSpeed = 50});
    toggle_preroller(true, 100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(2000);
    chassis.moveTo(-44, -55, 1000, {.forwards = false, .minSpeed = 60});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(32, -60, 270, 2500, {.forwards = false, .maxSpeed = 90, .minSpeed = 20});
    chassis.waitUntil(10);
    toggle_preroller(false);
    matchLoad.set_value(false);
    chassis.moveTo(45, -48, 1000, {.minSpeed = 40, .earlyExitRange = 9});
    chassis.moveTo(22, -49, 90, 1000, {.forwards = false});
    chassis.waitUntilDone();
    
    chassis.setPose(29.25, -48, 90); // CHAGE THETA TO THE CHASSIS VALUE
    toggle_score(true, 100, 100);
    pros::delay(1000);
    toggle_score(false);
    chassis.moveTo(59.5, -48, 90, 1500, {.maxSpeed = 60});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    matchLoad.set_value(true);
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveTo(23, -48, 90, 800, {.forwards = false, .minSpeed = 30});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(25);
    toggle_score(true);
    pros::delay(2000);
    toggle_score(false);
    matchLoad.set_value(false);
    chassis.moveTo(chassis.getPose().x + 5, chassis.getPose().y, 800);
    chassis.waitUntilDone();
    chassis.moveTo(chassis.getPose().x - 5, chassis.getPose().y, 800, {.forwards = false, .minSpeed = 40});
    chassis.waitUntilDone();
    
    chassis.setPose(29.25, -48, 90); // CHAGE THETA TO THE CHASSIS VALUE
    chassis.moveTo(38, -48, 800, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.waitUntilDone();
    chassis.moveTo(21, -21, 320, 2000, {.maxSpeed = 60});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    toggle_preroller(true, 90);
    chassis.turnTo(135, 500);
    chassis.moveTo(12, -10, 135, 1000, {.forwards = false});
    chassis.waitUntilDone();
    centerGoal.set_value(true);
    toggle_score(true, 80, 70);
    pros::delay(2000);
    toggle_score(false);
    centerGoal.set_value(false);
    chassis.moveTo(28, 4, 800, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.moveTo(45, 45, 2000, {.maxSpeed = 80});
    chassis.turnTo(90, 500);
    chassis.moveTo(24, 46, 90, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    */

    chassis.setPose(29.25, 48, 90); // CHAGE THETA TO THE CHASSIS VALUE
    matchLoad.set_value(true);
    pros::delay(100);
    chassis.moveTo(65, 48, 90, 1000, {.maxSpeed = 65});
    toggle_preroller(true, 100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.moveTo(44, 55, 1000, {.forwards = false, .minSpeed = 60});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-32, 60, 90, 2500, {.forwards = false, .maxSpeed = 90, .minSpeed = 20});
    chassis.waitUntil(10);
    toggle_preroller(false);
    matchLoad.set_value(false);
    chassis.moveTo(-45, 48, 1000, {.minSpeed = 40, .earlyExitRange = 9});
    chassis.moveTo(-22, 51, 270, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.setPose(-29.25, 48, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE

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
