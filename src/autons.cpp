#include "main.h"
#include "pahlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"

using namespace pahlib;

void matchLoadMove(int amt = 3){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    for (int i = 0; i < amt; i++) {
        chassis.tank(100, 100, true);
        pros::delay(300);
        chassis.tank(0, 0, true);
        pros::delay(300);
    }
}

/*
    Better way to set position
    x = TILE_LENGTH (140.25 / 6) * NUM_TILES_FROM_CENTER * DIRECTION_SIGN + OFFSET - DRIVE_WIDTH or DRIVE_LENGTH based on angle
    y = TILE_LENGTH (140.25 / 6) * NUM_TILES_FROM_CENTER * DIRECTION_SIGN + OFFSET - DRIVE_WIDTH or DRIVE_LENGTH based on angle
    // 23.54944882 is field tile length

*/

void auton1() {
    // SKILLS
    chassis.setPose(-46, 0.5, 180);
    antenne.set_value(true);
    pros::delay(0);
    chassis.moveTo(-46, -44, 2500, {.maxSpeed = 60});
    chassis.waitUntil(5);
    matchLoad.set_value(true);
    chassis.turnTo(270, 800);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-67, -46, 270, 900, {.lead = 0, .maxSpeed = 55});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    pros::delay(2500);
    chassis.moveTo(-22, -46.8, 270, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(100);
    toggle_score(true);
    matchLoad.set_value(false);
    pros::delay(2500);
    chassis.setPose(-29.25, -47, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE chassis.getPose().theta
    toggle_score(false);
    toggle_preroller(true);
    chassis.moveTo(-37, -47, 500);
    chassis.turnTo(5, 800);
    toggle_preroller(true);
    chassis.moveTo(-16.5, -15.5, 500);
    chassis.waitUntilDone();
    pros::delay(250);
    matchLoad.set_value(true);
    chassis.turnTo(0, 500);
    chassis.moveTo(-22, 35, 1000, {.maxSpeed = 70});
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    pros::delay(150);
    matchLoad.set_value(true);
    pros::delay(0);
    chassis.moveTo(-7.5, 6.5, 315, 1300, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(14);
    toggle_score(true, -40, -50);
    chassis.waitUntil(27.5);
    centerGoal.set_value(true);
    toggle_score(true, 100, -100);
    chassis.waitUntilDone();
    pros::delay(4000);
    chassis.moveTo(-41, 40, 800);
    chassis.waitUntil(3);
    centerGoal.set_value(false);
    toggle_score(false);
    chassis.moveTo(-70, 44.5, 270, 1500, {.maxSpeed = 60});
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(2500);
    chassis.moveTo(-23, 45.5, 270, 1500, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    toggle_score(true);
    matchLoad.set_value(false);
    pros::delay(2500);
    chassis.setPose(-29.25, 46, chassis.getPose().theta);
    chassis.moveTo(-59.5, 16, 180, 2000);
    chassis.turnTo(180, 500);
    chassis.waitUntilDone();
    doublePark.set_value(true); // lift up
    chassis.tank(90, 80, true);
    pros::delay(1100);
    chassis.tank(0, 0, true);

    /*
    chassis.setPose(-49.5, -15.5, 180);
    antenne.set_value(true);
    chassis.moveTo(-49.5, -44.2, 180, 1000, {.minSpeed = 40});
    chassis.waitUntil(10);
    matchLoad.set_value(true);
    chassis.turnTo(270, 500);
    toggle_preroller(true);
    chassis.moveTo(-62, -46, 270, 900, {.lead = 0, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveTo(chassis.getPose().x - 8, chassis.getPose().y, 500);
    pros::delay(1200);
    chassis.moveTo(-44, -55, 1000, {.forwards = false, .minSpeed = 60});
    chassis.moveTo(32, -61, 270, 2500, {.forwards = false, .maxSpeed = 90, .minSpeed = 20});
    chassis.waitUntil(10);
    matchLoad.set_value(false);
    toggle_preroller(false);
    chassis.turnTo(45, -47, 500);
    chassis.moveTo(45, -46, 1000, {.minSpeed = 40, .earlyExitRange = 8});
    matchLoad.set_value(true);
    toggle_score(true, -35, -80);
    chassis.turnTo(90, 500);
    toggle_score(false);
    chassis.moveTo(18, -47, 90, 1000, {.forwards = false});
    chassis.waitUntilDone();
    
    chassis.setPose(29.25, -48, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE
    toggle_score(true, 100, 100);
    pros::delay(2000);
    toggle_score(false);
    chassis.moveTo(62, -48, 90, 1500, {.lead = 0, .maxSpeed = 60});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveTo(20, -48, 90, 900, {.forwards = false, .minSpeed = 30});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(28);
    toggle_score(true);
    pros::delay(2000);
    toggle_score(false);
    matchLoad.set_value(false);
    chassis.setPose(29.25, -48, chassis.getPose().theta);
    chassis.moveTo(chassis.getPose().x + 5, chassis.getPose().y, 800);
    chassis.waitUntilDone();
    chassis.moveTo(chassis.getPose().x - 8, chassis.getPose().y, 800, {.forwards = false, .minSpeed = 80});
    chassis.waitUntilDone();
    
    //chassis.setPose(29.25, -48, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE
    chassis.moveTo(38, -48, 800, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.waitUntilDone();
    chassis.moveTo(18, -22, 320, 2000, {.maxSpeed = 60});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    toggle_preroller(true, 90);
    chassis.turnTo(135, 500);
    matchLoad.set_value(true);
    chassis.moveTo(6, -5, 135, 1000, {.forwards = false});
    toggle_score(true, -50, -80);
    chassis.waitUntil(10);
    toggle_score(false);
    chassis.waitUntilDone();
    centerGoal.set_value(true);
    toggle_score(true, 80, 80);
    pros::delay(2000);
    matchLoad.set_value(false);
    chassis.moveTo(12, -12, 500, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.moveTo(28, 4, 800, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.moveTo(45, 48, 2500, {.maxSpeed = 65});
    toggle_score(false);
    centerGoal.set_value(false);
    chassis.turnTo(90, 500);
    chassis.moveTo(15, 49, 90, 900, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    
    chassis.setPose(29.25, 48, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE
    matchLoad.set_value(true);
    pros::delay(100);
    chassis.moveTo(75, 48, 90, 1000, {.maxSpeed = 70});
    toggle_preroller(true, 100);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    pros::delay(600);
    chassis.moveTo(chassis.getPose().x + 5, chassis.getPose().y, 400);
    pros::delay(1200);
    chassis.moveTo(44, 58, 1000, {.forwards = false, .minSpeed = 60});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-35, 63, 90, 2500, {.forwards = false, .maxSpeed = 90, .minSpeed = 20});
    chassis.waitUntil(10);
    toggle_preroller(false);
    matchLoad.set_value(false);
    chassis.moveTo(-45, 48, 1000, {.minSpeed = 40, .earlyExitRange = 9});
    chassis.waitUntil(7);
    matchLoad.set_value(true);
    toggle_score(true, -35, -80);
    chassis.turnTo(270, 500);
    toggle_score(false);
    chassis.moveTo(-20, 48, 270, 1000, {.forwards = false});
    chassis.waitUntilDone();

    chassis.setPose(-29.25, 48, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE
    toggle_score(true, 100, 100);
    pros::delay(2000);
    toggle_score(false);
    chassis.moveTo(-67, 48, 270, 1500, {.maxSpeed = 60});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    matchLoad.set_value(true);
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveTo(-20, 48, 270, 900, {.forwards = false, .minSpeed = 30});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(28);
    toggle_score(true);
    pros::delay(1200);
    toggle_score(false);
    matchLoad.set_value(false);
    chassis.moveTo(chassis.getPose().x - 5, chassis.getPose().y, 800);
    chassis.waitUntilDone();
    chassis.moveTo(chassis.getPose().x + 5, chassis.getPose().y, 800, {.forwards = false, .minSpeed = 40});
    chassis.waitUntilDone();
    
    //chassis.setPose(-29.25, 48, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE
    chassis.moveTo(-38, 48, 800, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.waitUntilDone();
    chassis.moveTo(-21, 21, 2000, {.maxSpeed = 60});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    toggle_preroller(true, 90);
    chassis.turnTo(315, 500);
    matchLoad.set_value(true);
    chassis.moveTo(-6.1, 5.9, 315, 1000, {.forwards = false, .maxSpeed = 50});
    toggle_score(true, -35, -80);
    chassis.waitUntil(10);
    toggle_score(false);
    chassis.waitUntilDone();
    centerGoal.set_value(true);
    toggle_score(true, 80, 80);
    pros::delay(2000);
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    chassis.moveTo(-75, 27, 1000);
    toggle_score(false);
    chassis.waitUntilDone();
    chassis.turnTo(20, 500);
    centerGoal.set_value(false);
    chassis.waitUntilDone();
    doublePark.set_value(true); // turn off odom
    chassis.cancelAllMotions();
    chassis.tank(-100, -100, true);
    //chassis.moveTo(chassis.getPose().x - 20, chassis.getPose().y, 1500);
    pros::delay(2000);
    chassis.tank(0, 0, true);
    */
}

void auton2() {
    // SAWP
    chassis.setPose(-46, 0.5, 180);
    antenne.set_value(true);
    //chassis.moveTo(-46, 8, 500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 1});
    pros::delay(0);
    chassis.moveTo(-46, -40, 2000, {.maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 2});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(5);
    matchLoad.set_value(true);
    chassis.turnTo(270, 800, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-65, -46.5, 270, 900, {.lead = 0, .maxSpeed = 55});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveTo(-27, -46.8, 270, 1000, {.forwards = false});
    chassis.waitUntil(8);
    toggle_score(true, 80, 10);
    chassis.waitUntil(35);
    toggle_score(true);
    chassis.waitUntilDone();
    matchLoad.set_value(false);
    pros::delay(800);
    chassis.setPose(-29.25, -47, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE chassis.getPose().theta
    toggle_score(false);
    toggle_preroller(true);
    chassis.moveTo(-37, -47, 500, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.turnTo(5, 800, {.minSpeed = 10, .earlyExitRange = 3});
    toggle_preroller(true);
    chassis.moveTo(-16.5, -15.5, 500);
    chassis.waitUntilDone();
    pros::delay(250);
    matchLoad.set_value(true);
    chassis.turnTo(0, 500);
    chassis.moveTo(-22, 35, 1000, {.maxSpeed = 70});
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    pros::delay(150);
    matchLoad.set_value(true);
    pros::delay(0);
    chassis.moveTo(-7.5, 9, 315, 1300, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(14);
    toggle_score(true, -25, -50);
    chassis.waitUntil(27.5);
    centerGoal.set_value(true);
    toggle_score(true, 100, -100);
    pros::delay(1000);
    chassis.moveTo(-42, 43, 800, {.minSpeed = 5, .earlyExitRange = 5});
    chassis.waitUntil(3);
    centerGoal.set_value(false);
    toggle_score(false);
    //matchLoad.set_value(false); // REMOVE LATER
    chassis.moveTo(-65, 45.5, 270, 1500, {.maxSpeed = 60});
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveTo(-23, 46, 270, 1000, {.forwards = false});
    chassis.waitUntil(8);
    toggle_score(true, 80, 10);
    chassis.waitUntil(35);
    toggle_score(true);
    chassis.waitUntilDone();
    matchLoad.set_value(false);
    /*
    antenne.set_value(true);
    chassis.setPose(-47.75, -13.75, 90);
    toggle_preroller(true, 90);
    chassis.turnTo(-15, -24, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-17, -26, 115, 1000, {.minSpeed = 10});
    chassis.waitUntil(30);
    chassis.turnTo(-67, -46, 500);
    chassis.moveTo(-45, -41, 500, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.moveTo(-45, -42, 500, {.minSpeed = 80, .earlyExitRange = 5});
    //toggle_preroller(false);
    matchLoad.set_value(true);
    chassis.turnTo(270, 500, {.maxSpeed = 80});
    chassis.moveTo(-68, -46, 270, 1000, {.lead = 0, .maxSpeed = 80});
    toggle_preroller(true, 90);
    chassis.waitUntilDone();
    chassis.moveTo(chassis.getPose().x - 2, chassis.getPose().y, 200);
    chassis.moveTo(-15, -47.5, 270, 1000, {.forwards = false, .maxSpeed = 90});
    toggle_score(true, -40, -80);
    chassis.waitUntil(15);
    toggle_score(false);
    chassis.waitUntil(35);
    toggle_score(true);
    pros::delay(1900);
    chassis.setPose(-29.25, -48, chassis.getPose().theta);
    toggle_score(false);
    chassis.moveTo(-38, -36.8, 500, {.maxSpeed = 80});
    matchLoad.set_value(false);
    chassis.moveTo(-6, -36.7, 270, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2});
    chassis.waitUntil(5);
    antenne.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnTo(245, 350, {.minSpeed = 80});
    */
}

void auton3() {
    // RIGHT
    chassis.setPose(-46, -14.5, 90);
    antenne.set_value(true);
    chassis.turnTo(-22, -22.5, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-23, -22, 1000, {.minSpeed = 20});
    toggle_preroller(true);
    chassis.waitUntil(15);
    matchLoad.set_value(true);
    chassis.turnTo(70, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-44, -47, 0, 2000, {.forwards = false, .minSpeed = 40});
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-65, -46, 270, 1200, {.lead = 0, .maxSpeed = 55});
    chassis.waitUntilDone();
    pros::delay(300);
    chassis.moveTo(-22, -45.5, 270, 1000, {.forwards = false});
    chassis.waitUntilDone();
    toggle_score(true);
    pros::delay(100);
    matchLoad.set_value(false);
    pros::delay(1500);
    chassis.setPose(-29.25, -47, chassis.getPose().theta);
    toggle_score(false);
    chassis.moveTo(-38, -36, 500, {.maxSpeed = 80});
    matchLoad.set_value(false);
    chassis.moveTo(-13, -36.2, 270, 1500, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});
    chassis.waitUntil(5);
    antenne.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnTo(245, 350, {.minSpeed = 80});
    /*
    chassis.setPose(-47.75, 13.75, 90);
    toggle_preroller(true);
    //chassis.turnTo(50, 500, {.earlyExitRange = 3});
    chassis.moveTo(-16, 29, 50, 1000, {.maxSpeed = 90});
    chassis.moveTo(-9.8, 45, 800, {.maxSpeed = 70, .minSpeed = 30});
    chassis.waitUntil(3);
    toggle_preroller(true, 40);
    chassis.waitUntil(7);
    toggle_preroller(true, 90);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveTo(-23, 20.6, 1000, {.forwards = false});
    toggle_preroller(false);
    chassis.turnTo(310, 500);
    chassis.moveTo(-7.4, 4.1, 315, 1000, {.forwards = false, .minSpeed = 50});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    toggle_score(true, -15, -80);
    matchLoad.set_value(true);
    chassis.waitUntilDone();
    centerGoal.set_value(true);
    toggle_score(true, 90, 90);
    pros::delay(900);

    toggle_score(false);
    chassis.moveTo(-48, 45, 1500, {.maxSpeed = 70});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    toggle_score(true, -35, -80);
    chassis.waitUntil(5);
    centerGoal.set_value(false);
    chassis.waitUntil(8);
    toggle_score(false);
    chassis.turnTo(270, 500);
    matchLoad.set_value(true);
    chassis.moveTo(-70, 48, 270, 700, {.lead = 0, .maxSpeed = 90});
    toggle_preroller(true, 90);
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.moveTo(0, 48.3, 270, 1000, {.forwards = false, .minSpeed = 20});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    pros::delay(100);
    toggle_score(true);
    matchLoad.set_value(false);
    pros::delay(1600);
    chassis.setPose(-29.25, 48, chassis.getPose().theta);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-38, 61, 500, {.maxSpeed = 60});
    toggle_score(false);
    matchLoad.set_value(false);
    chassis.moveTo(-4, 60, 270, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2});
    chassis.waitUntil(5);
    antenne.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnTo(245, 350, {.minSpeed = 80});
    */
}

void auton4() {
// SAWP
    chassis.setPose(-49.5, -15.5, 180);
    antenne.set_value(true);
    chassis.moveTo(-49.5, -46, 180, 1000, {.minSpeed = 30});
    chassis.waitUntil(10);
    chassis.turnTo(270, 500, {.maxSpeed = 60});
    toggle_preroller(true);
    matchLoad.set_value(true);
    chassis.moveTo(-63, -46, 270, 900, {.lead = 0, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveTo(chassis.getPose().x - 5, chassis.getPose().y, 100);
    pros::delay(100);
    chassis.moveTo(-20, -47.4, 270, 1000, {.forwards = false, .lead = 0, .minSpeed = 60});
    chassis.waitUntil(32);
    matchLoad.set_value(false);
    toggle_score(true, 127, 127);
    pros::delay(750);
    chassis.setPose(-29.25, -48, chassis.getPose().theta);
    chassis.turnTo(14, 800);
    toggle_score(false);
    toggle_preroller(true);
    chassis.moveTo(-24, -19, 1000, {.maxSpeed = 65});
    chassis.moveTo(-25, 16, 2000, {.maxSpeed = 65});
    chassis.waitUntilDone();
    matchLoad.set_value(true);
    chassis.moveTo(-7.4, 3.4, 315, 1000, {.forwards = false, .minSpeed = 40});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    toggle_score(true, -35, -80);
    chassis.waitUntilDone();
    toggle_score(true, 90, 90);
    centerGoal.set_value(true);
    pros::delay(1000);
    toggle_score(true, -35, -80);
    pros::delay(200);
    chassis.moveTo(-47, 45, 1500, {.maxSpeed = 70});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(5);
    centerGoal.set_value(false);
    chassis.waitUntil(8);
    toggle_score(false);
    chassis.turnTo(270, 500, {.maxSpeed = 60});
    toggle_preroller(true);
    chassis.moveTo(-62, 47, 270, 1000, {.lead = 0, .maxSpeed = 70});
    pros::delay(100);
    chassis.moveTo(-15, 47, 270, 1000, {.forwards = false, .minSpeed = 40});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(40);
    toggle_score(true, 127, 127);
    pros::delay(500);
    matchLoad.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(800);
    chassis.moveTo(chassis.getPose().x - 5, chassis.getPose().y, 500);
}

void auton5() {
    //RIGHT + LOWER
    chassis.setPose(-47.75, -13.75, 90);
    toggle_preroller(true, 90);
    chassis.turnTo(-15, -24, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-17, -25, 115, 1000, {.minSpeed = 30});
    chassis.moveTo(-4.5, -45, 800, {.maxSpeed = 70, .minSpeed = 30});
    chassis.waitUntil(3);
    toggle_preroller(true, 40);
    chassis.waitUntil(7);
    toggle_preroller(true, 90);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveTo(-24, -26, 1000, {.forwards = false});
    toggle_preroller(false);
    chassis.turnTo(40, 500);
    chassis.moveTo(-18, -13, 45, 1000, {.minSpeed = 30});
    chassis.turnTo(45, 500);
    chassis.waitUntilDone();
    toggle_score(true, -80, -60);
    pros::delay(1200);
    toggle_score(false);
    //
    chassis.moveTo(-44, -45, 1000, {.forwards = false, .minSpeed = 5});
    //toggle_preroller(false);
    chassis.turnTo(270, 800);
    matchLoad.set_value(true);
    toggle_preroller(true);
    chassis.moveTo(-65, -48, 270, 800, {.lead = 0, .maxSpeed = 80});
    chassis.moveTo(chassis.getPose().x - 2, chassis.getPose().y, 200);
    pros::delay(200);
    chassis.moveTo(-20, -49, 270, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntil(40);
    matchLoad.set_value(false);
    toggle_score(true);
    pros::delay(900);
    chassis.setPose(-29.25, -48, chassis.getPose().theta);
    toggle_score(false);
    chassis.moveTo(-38, -37, 500, {.maxSpeed = 80});
    matchLoad.set_value(false);
    chassis.moveTo(-6, -37.2, 270, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2});
    chassis.waitUntil(5);
    antenne.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnTo(245, 350, {.minSpeed = 80});
}

void auton6() {
    chassis.setPose(-49.5, -15.5, 180);
    antenne.set_value(true);
    chassis.moveTo(-49.5, -44.5, 180, 1000, {.minSpeed = 40});
    chassis.waitUntil(10);
    matchLoad.set_value(true);
    chassis.turnTo(270, 500);
    toggle_preroller(true);
    chassis.moveTo(-62, -45.5, 270, 900, {.lead = 0, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveTo(chassis.getPose().x - 5, chassis.getPose().y, 100);
    pros::delay(100);
    chassis.moveTo(-20, -46, 270, 1000, {.forwards = false, .lead = 0, .minSpeed = 60});
    chassis.waitUntil(35);
    matchLoad.set_value(false);
    toggle_score(true, 127, 127);
    pros::delay(750);
    chassis.setPose(-29.25, -48, chassis.getPose().theta);
    chassis.turnTo(12, 800);
    toggle_score(false);
    toggle_preroller(true);
    chassis.moveTo(-25.5, -32, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.moveTo(-25.5, 16, 2000, {.maxSpeed = 65});
    chassis.waitUntilDone();
    matchLoad.set_value(true);
    chassis.turnTo(315, 500);
    chassis.moveTo(-6, 2, 315, 1000, {.forwards = false, .minSpeed = 40});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    toggle_score(true, -35, -80);
    chassis.waitUntilDone();
    toggle_score(true, 90, 85);
    centerGoal.set_value(true);
    pros::delay(1000);
    toggle_score(true, -35, -80);
    pros::delay(200);
    chassis.moveTo(-47, 32, 1500, {.maxSpeed = 80});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(5);
    centerGoal.set_value(false);
    chassis.waitUntil(8);
    toggle_score(false);
    chassis.turnTo(270, 500);
    toggle_preroller(true);
    chassis.moveTo(-67, 33, 270, 1000, {.lead = 0, .maxSpeed = 70});
    pros::delay(200);
    chassis.moveTo(-15, 36, 270, 1000, {.forwards = false, .minSpeed = 40});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(40);
    toggle_score(true, 127, 127);
    pros::delay(500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

void auton7() {
    chassis.resetOdometry();
}

void auton8() {
    
}

void auton9() {
    
}

void auton10() {
    
}
