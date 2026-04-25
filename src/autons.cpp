#include "main.h"
#include "pahlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"

using namespace pahlib;

void matchLoadMove(int amt = 6){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    for (int i = 0; i < amt; i++) {
        chassis.tank(40, 40, true);
        pros::delay(200);
    }
}

void fourBallArc(int side){
    chassis.moveTo(-32, 47 * side, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.tank(side > 0 ? 0 : -100, side > 0 ? -100 : 0, true);
    pros::delay(450);
    chassis.tank(0, 0, true);
    chassis.moveTo(-10, 47 * side, 270, 800, {.forwards = false, .lead = 0, .minSpeed = 20, .earlyExitRange = 3});
    pros::delay(200);
}

void goalPush(int side){
    if (side > 0){
        chassis.moveTo(-38, -36, 500, {.minSpeed = 40});
        matchLoad.set_value(false);
        chassis.moveTo(-9, -37, 270, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2});
        chassis.waitUntil(5);
        antenne.set_value(false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.turnTo(245, 350, {.minSpeed = 50});
    } else {
        chassis.moveTo(-38, 58.5, 500, {.minSpeed = 40});
        matchLoad.set_value(false);
        chassis.moveTo(-9, 57.5, 270, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2});
        chassis.waitUntil(5);
        antenne.set_value(false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.turnTo(245, 350, {.minSpeed = 50});
    }
}

/*
    Better way to set position
    x = TILE_LENGTH (140.25 / 6) * NUM_TILES_FROM_CENTER * DIRECTION_SIGN + OFFSET - DRIVE_WIDTH or DRIVE_LENGTH based on angle
    y = TILE_LENGTH (140.25 / 6) * NUM_TILES_FROM_CENTER * DIRECTION_SIGN + OFFSET - DRIVE_WIDTH or DRIVE_LENGTH based on angle
    // 23.54944882 is field tile length

    DISTANCE RESET
    reset.updateBotPose(&left_rcl);   // Distance reset on the left sensor
    reset.updateBotPose();            // Update chassis position based on RCL
    reset.setRclPose(chassis.getPose());  // Reset Rcl Pose to Lemlib Pose

*/

/*

Order of autons:
1. Skills
2. Counter SAWP
3. Fast Right 4
4. Right 7
5. Right 9 Tech
6. Right 9 Split
7. Fast Left 4
8. Left 7
9. Left 7 Split
10. Left 7 Center First

*/

// Skills
void auton1() {
    // SKILLS
    chassis.setPose(-46, 14.5, 90);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    chassis.moveTo(-25.3, 19.5, 65, 1000, {.minSpeed = 5}); // 61, remove lead
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(14);
    toggle_preroller(true, 120);
    matchLoad.set_value(true);
    chassis.waitUntilDone();
    pros::delay(100);
    toggle_preroller(true);
    chassis.moveTo(-27.5, 22, 500);
    chassis.turnTo(315, 500);
    chassis.moveTo(-8.3, 7.3, 315, 1000, {.forwards = false});
    chassis.waitUntil(10);
    toggle_preroller(false);
    toggle_score(true, -10, -50);
    chassis.waitUntilDone();
    centerGoal.set_value(true);
    chassis.tank(-35, -35, true);
    toggle_score(true, 100, -120);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(400);
    chassis.tank(0, 0, true);
    chassis.moveTo(-52, 48, 315, 1000);
    chassis.waitUntil(4);
    toggle_score(true, -30, 60);
    chassis.waitUntil(8);
    toggle_score(false);
    centerGoal.set_value(false);
    toggle_preroller(true);
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    reset.updateBotPose(&right_rcl);
    pros::delay(50);
    chassis.moveTo(-10, 47, 270, 800, {.forwards = false, .minSpeed = 20});
    chassis.waitUntilDone();
    chassis.turnTo(270, 400, {.minSpeed = 40});
    chassis.tank(-30, -30, true);
    toggle_score(true, 127, 127);
    score.set_value(true);
    pros::delay(700);
    chassis.setPose(-29.5, 47, chassis.getPose().theta);
    reset.setRclPose(chassis.getPose());
    chassis.tank(0, 0, true);
    chassis.moveTo(-70, 47, 800, {.minSpeed = 10, .earlyExitRange = 35});
    chassis.moveTo(-70, 47, 270, 800, {.maxSpeed = 53});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    score.set_value(false);
    toggle_score(false);
    toggle_preroller(true, 127);
    chassis.waitUntilDone();
    matchLoadMove();
    pros::delay(200);
    chassis.moveTo(-34, 59, 800, {.forwards = false, .minSpeed = 20, .earlyExitRange = 5});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    toggle_preroller(true);
    chassis.moveTo(34, 61, 270, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
    toggle_preroller(false);
    chassis.moveTo(39, 48.5, 800, {.forwards = false});
    chassis.turnTo(90, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(13, 47, 90, 1000, {.forwards = false});
    toggle_preroller(true);
    chassis.waitUntil(20);
    toggle_score(true, -20, -30);
    chassis.turnTo(90, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    chassis.tank(-40, -40, true);
    toggle_score(true, 127, 127);
    score.set_value(true);
    pros::delay(1400);
    chassis.setPose(29.5, 47, chassis.getPose().theta);
    reset.setRclPose(chassis.getPose());
    chassis.tank(0, 0, true);
    score.set_value(false);
    toggle_score(false);
    toggle_preroller(true);
    
    chassis.setPose(29.5, 47, chassis.getPose().theta);
    chassis.moveTo(70, 47, 1500, {.minSpeed = 10, .earlyExitRange = 35});
    chassis.moveTo(70, 47, 270, 800, {.maxSpeed = 53});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    matchLoadMove(7);
    toggle_preroller(true, 127);
    pros::delay(200);
    chassis.moveTo(15, 47, 90, 1000, {.forwards = false});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    toggle_preroller(true, -10);
    chassis.waitUntil(15);
    toggle_preroller(false);
    chassis.waitUntilDone();
    chassis.tank(-40, -40, true);
    chassis.turnTo(90, 500, {.minSpeed = 40});
    score.set_value(true);
    toggle_score(true, 127, 127);
    pros::delay(1500);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    chassis.setPose(29.5, 47, chassis.getPose().theta);

    // sawp part
    chassis.moveTo(37, 47, 500, {.minSpeed = 40, .earlyExitRange = 5});
    score.set_value(false);
    chassis.turnTo(185, 800, {.minSpeed = 20, .earlyExitRange = 3});
    toggle_score(true, 80, 80);
    chassis.moveTo(16, 15.5, 500, {.minSpeed = 20});
    toggle_score(false);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(175);
    matchLoad.set_value(true);
    //chassis.turnTo(0, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(19.5, -25, 1400, {.maxSpeed = 70, .minSpeed = 10}); //max = 70
    pros::delay(350);
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    pros::delay(5);
    matchLoad.set_value(true);
    pros::delay(0);
    chassis.turnTo(135, 500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveTo(10, -8, 135, 900, {.forwards = false, .lead = 0, .maxSpeed = 75, .minSpeed = 15});
    chassis.waitUntil(14);
    toggle_score(true, -70, -70);
    chassis.waitUntil(27.5);
    centerGoal.set_value(true);
    chassis.waitUntilDone();
    chassis.tank(-20, -20, true);
    toggle_score(true, 0, 120);
    pros::delay(500);
    toggle_score(true, 100, -120);
    pros::delay(2000);
    chassis.moveTo(40, -45, 1000);
    chassis.waitUntil(1);
    toggle_score(true, -35, 0);
    chassis.waitUntil(20);
    toggle_preroller(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnTo(90, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    reset.updateBotPose(&right_rcl);
    pros::delay(50);
    chassis.moveTo(15, -46.8, 270, 750, {.forwards = false, .maxSpeed = 85, .minSpeed = 25});
    chassis.waitUntilDone();
    chassis.turnTo(90, 500);
    score.set_value(true);
    toggle_score(true, 127, 127);
    pros::delay(300);
    centerGoal.set_value(false);
    pros::delay(1700);

    /*
    //Center
    score.set_value(false);
    matchLoad.set_value(false);
    toggle_score(false);
    pros::delay(50);
    toggle_score(true, 70, 70);
    chassis.tank(50, 50, true);
    pros::delay(300);
    score.set_value(false);
    chassis.tank(0, 0, true);
    chassis.moveTo(60, 30, 900, {.maxSpeed = 80});
    chassis.turnTo(180, 500, {.maxSpeed = 100});
    chassis.moveTo(63.5, 16, 900, {.maxSpeed = 70});
    chassis.turnTo(178, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    
    doublePark.set_value(true);
    toggle_score(false);
    chassis.tank(-40, -40, true);
    pros::delay(200);
    chassis.tank(65, 65, true);
    pros::delay(200);
    chassis.tank(75, 85, true);
    toggle_preroller(true, 115);
    pros::delay(600);
    chassis.tank(90, 95, true);
    pros::delay(500);
    chassis.tank(75, 85, true);
    pros::delay(400);
    matchLoad.set_value(true);
    chassis.tank(60, 70, true);
    pros::delay(300);
    matchLoad.set_value(false);
    pros::delay(1500);
    chassis.tank(0, 0, true);
    doublePark.set_value(false);
    chassis.setPose(65, -20, chassis.getPose().theta);
    chassis.turnTo(180, 500);
    chassis.waitUntilDone();
    chassis.tank(-40, -50, true);
    pros::delay(1000);
    chassis.tank(0, 0, true);
    chassis.setPose(chassis.getPose().x, -16.25, chassis.getPose().theta);
    reset.setRclPose(chassis.getPose());
    pros::delay(100);
    chassis.tank(20, 20, true);
    pros::delay(200);
    chassis.tank(0, 0, true);
    chassis.turnTo(270, 1000, {.minSpeed = 20});
    toggle_preroller(true, -30);
    pros::delay(100);
    toggle_preroller(false);
    chassis.waitUntilDone();
    chassis.tank(-60, -60, true);
    pros::delay(450);
    chassis.tank(0, 0, true);
    chassis.setPose(62, chassis.getPose().y, 270);
    reset.setRclPose(chassis.getPose());
    pros::delay(50);
    chassis.moveTo(41, -12, 800, {.minSpeed = 10, .earlyExitRange = 3});
    //toggle_score(true, 90, -30);
    chassis.moveTo(26, -20.5, 225, 1000);
    toggle_preroller(true);
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    chassis.tank(-50, -10, true);
    pros::delay(350);
    chassis.tank(0, 0, true);
    //matchLoad.set_value(true);
    chassis.turnTo(135, 500);
    chassis.moveTo(10.7, -6.5, 135, 1000, {.forwards = false, .lead = 0, .maxSpeed = 60});
    toggle_preroller(false);
    toggle_score(true, -40, -80);
    pros::delay(300);
    matchLoad.set_value(true);
    toggle_score(false);
    centerGoal.set_value(true);
    chassis.turnTo(140, 500, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.tank(20, -90, true);
    pros::delay(400);
    chassis.tank(-40, -40, true);
    pros::delay(100);
    toggle_score(true, 90, -120);
    pros::delay(200);
    chassis.tank(0, 0, true);
    chassis.turnTo(135, 500);
    chassis.waitUntilDone();
    chassis.tank(-40, -40, true);
    pros::delay(300);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    for (int i = 0; i < 4; i++) {
        chassis.tank(-25, -25, true);
        pros::delay(200);
        chassis.tank(5, 5, true);
        pros::delay(200);
    }
    chassis.tank(-40, -40, true);
    pros::delay(400);
    chassis.tank(0, 0, true);
    toggle_score(false);
    matchLoad.set_value(false);
    toggle_preroller(true);
    chassis.moveTo(52, -49, 1000, {.maxSpeed = 90});
    chassis.waitUntil(5);
    centerGoal.set_value(false);
    chassis.turnTo(90, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    reset.updateBotPose(&right_rcl);
    pros::delay(50);
    chassis.moveTo(10, -47, 90, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.tank(-40, -40, true);
    chassis.turnTo(90, 400, {.minSpeed = 40});
    toggle_score(true, 127, 127);
    score.set_value(true);
    matchLoad.set_value(true);
    pros::delay(600);
    chassis.tank(0, 0, true);
    //
    */
    // END PART
    chassis.setPose(29.5, -47, chassis.getPose().theta); // CHANGE THETA TO THE CHASSIS VALUE
    chassis.tank(0, 0, true);
    matchLoad.set_value(true);
    chassis.moveTo(70, -47, 1500, {.minSpeed = 10, .earlyExitRange = 35});
    chassis.moveTo(70, -47, 270, 800, {.maxSpeed = 53});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    score.set_value(false);
    toggle_score(false);
    toggle_preroller(true, 127);
    chassis.waitUntilDone();
    matchLoadMove(7);
    pros::delay(200);
    chassis.moveTo(34, -59, 800, {.forwards = false, .minSpeed = 20, .earlyExitRange = 5});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    toggle_preroller(true);
    chassis.moveTo(-30, -61, 270, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
    toggle_preroller(false);
    chassis.moveTo(-33, -48, 800, {.forwards = false});
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    reset.updateBotPose(&left_rcl);
    pros::delay(50);
    chassis.moveTo(-15, -47, 270, 1000, {.forwards = false});
    toggle_preroller(true);
    chassis.waitUntil(20);
    toggle_score(-20, -30);
    chassis.turnTo(270, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    chassis.tank(-40, -40, true);
    toggle_score(true, 127, 127);
    score.set_value(true);
    pros::delay(1400);
    chassis.tank(0, 0, true);
    score.set_value(false);
    toggle_score(false);
    toggle_preroller(true);
    chassis.setPose(-29.5, -47, chassis.getPose().theta);
    chassis.moveTo(-70, -47, 1500, {.minSpeed = 10, .earlyExitRange = 35});
    chassis.moveTo(-74, -47, 270, 800, {.maxSpeed = 53});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    matchLoadMove(7);
    toggle_preroller(true, 127);
    pros::delay(400);
    chassis.moveTo(-15, -47, 90, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    toggle_preroller(true, -10);
    chassis.waitUntil(15);
    toggle_preroller(false);
    chassis.waitUntilDone();
    chassis.tank(-35, -35, true);
    chassis.turnTo(270, 500, {.minSpeed = 40});
    score.set_value(true);
    toggle_score(true, 127, 127);
    pros::delay(1500);
    chassis.tank(0, 0, true);
    chassis.setPose(-29.5, -47, chassis.getPose().theta); // CHANGE THETA TO THE CHASSIS VALUE
    toggle_score(false);
    matchLoad.set_value(false);
    score.set_value(true);
    toggle_preroller(true, 70);
    chassis.tank(50, 50, true);
    pros::delay(300);
    chassis.tank(0, 0, true);
    chassis.moveTo(-61, -30, 800, {.maxSpeed = 90});
    chassis.turnTo(0, 500, {.maxSpeed = 100});
    chassis.moveTo(-63.5, -15, 900, {.maxSpeed = 70});
    chassis.turnTo(0, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    chassis.tank(30, 30, true);
    doublePark.set_value(true);
    toggle_score(false);
    pros::delay(200);
    chassis.tank(-40, -40, true);
    pros::delay(200);
    chassis.tank(65, 65, true);
    pros::delay(200);
    chassis.tank(75, 80, true);
    toggle_preroller(true, 115);
    pros::delay(1300);
    chassis.tank(0, 0, true);
    score.set_value(false);
}

// Counter Sawp
void auton2() {
    // COUNTER SAWP
    chassis.setPose(-46, 0.5, 180);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    pros::delay(0);
    chassis.tank(-50, -50, true);
    pros::delay(300);
    chassis.tank(0, 0, true);
    chassis.moveTo(-46, -33, 2000, {.maxSpeed = 100, .minSpeed = 25, .earlyExitRange = 3});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(5);
    matchLoad.set_value(true);
    chassis.waitUntilDone();
    //chassis.resetOdometry();
    chassis.turnTo(270, 800, {.minSpeed = 20, .earlyExitRange = 3}); //comment out?
    chassis.waitUntilDone();
    reset.updateBotPose(&left_rcl);
    pros::delay(50);
    chassis.moveTo(-78, -47, 270, 900, {.lead = 0, .maxSpeed = 53});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    reset.updateBotPose(&left_rcl);
    pros::delay(450);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, -47, 700, {.forwards = false, .maxSpeed = 85, .minSpeed = 25});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(40);
    score.set_value(true);
    toggle_score(true, 127, 127);
    chassis.waitUntilDone();
    matchLoad.set_value(false);
    chassis.tank(-35, -35, true);
    pros::delay(700);
    chassis.tank(0, 0, true);
    chassis.setPose(-29.5, -47, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE chassis.getPose().theta
    toggle_score(false);
    toggle_preroller(true);

    chassis.moveTo(-37, -47, 500, {.minSpeed = 40, .earlyExitRange = 5});
    score.set_value(false);
    chassis.turnTo(5, 800, {.minSpeed = 20, .earlyExitRange = 3});
    toggle_score(true, 80, 80);
    chassis.moveTo(-16, -15.5, 500, {.minSpeed = 20});
    toggle_score(false);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(175);
    matchLoad.set_value(true);
    //chassis.turnTo(0, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-19.5, 25, 1400, {.maxSpeed = 70, .minSpeed = 10}); //max = 70
    pros::delay(350);
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    pros::delay(5);
    matchLoad.set_value(true);
    pros::delay(0);
    chassis.moveTo(-40, 40, 1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    reset.updateBotPose(&right_rcl);
    pros::delay(50);
    chassis.moveTo(-15, 46.8, 270, 750, {.forwards = false, .maxSpeed = 85, .minSpeed = 25});
    chassis.waitUntilDone();
    chassis.turnTo(270, 500);
    score.set_value(true);
    toggle_score(true, 127, 127);
    pros::delay(900);
    chassis.setPose(-29.25, 47, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE chassis.getPose().theta
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-78, 47, 1400, {.minSpeed = 70, .earlyExitRange = 35});
    chassis.moveTo(-78, 47, 270, 500, {.maxSpeed = 53});
    toggle_score(false);
    toggle_preroller(true);
    score.set_value(false);
    chassis.waitUntilDone();
    chassis.tank(35, 35, true);
    pros::delay(200);
    chassis.tank(0, 0, true);
    chassis.moveTo(chassis.getPose().x + 8, chassis.getPose().y - 2, 500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-8.8, 8, 315, 1500, {.forwards = false, .minSpeed = 10});
    chassis.waitUntil(15);
    toggle_score(true, -40, -80);
    pros::delay(300);
    toggle_score(false);
    centerGoal.set_value(true);
    chassis.waitUntilDone();
    toggle_score(true, 95, -105);
    matchLoad.set_value(false);
    chassis.tank(-15, -15, true);
}

// Fast Right 4
void auton3() {
    chassis.setPose(-46, -14.5, 90);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    chassis.moveTo(-26.8, -21, 1000, {.minSpeed = 30});
    toggle_preroller(true);
    chassis.waitUntil(13);
    matchLoad.set_value(true);
    pros::delay(100);
    fourBallArc(-1);
    toggle_score(true, 120, 120);
    score.set_value(true);
    pros::delay(800);
    chassis.setPose(-29.5, -47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    toggle_score(false);
    score.set_value(false);
    goalPush(1);
}

// Right 7
void auton4() {
    // RIGHT 7
    chassis.setPose(-46, -14.5, 90);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    //chassis.turnTo(-22, -22.5, 500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveTo(-26.8, -21, 1000, {.minSpeed = 30});
    toggle_preroller(true);
    chassis.waitUntil(15);
    matchLoad.set_value(true);
    pros::delay(100);
    //chassis.turnTo(250, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-45, -40, 2000, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.moveTo(-75, -47, 270, 800, {.lead = 0, .maxSpeed = 58});
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(250);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, -47, 700, {.forwards = false, .minSpeed = 35});
    chassis.waitUntil(30);
    toggle_score(true, -40, -70);
    chassis.waitUntilDone();
    toggle_score(true, 127, 127);
    matchLoad.set_value(false);
    chassis.tank(-40, -40, true);
    score.set_value(true);
    pros::delay(1200);
    chassis.setPose(-29.5, -47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    toggle_score(false);
    score.set_value(false);
    goalPush(1);
}

// Right 9 Tech
void auton5(){
    // RIGHT 7 + Tech
    chassis.setPose(-46, -14.5, 90);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    //chassis.turnTo(-22, -22.5, 500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveTo(-26.8, -21, 1000, {.minSpeed = 30});
    toggle_preroller(true);
    chassis.waitUntil(15);
    matchLoad.set_value(true);
    pros::delay(100);
    //chassis.turnTo(250, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-45, -42, 2000, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.moveTo(-75, -47, 270, 800, {.lead = 0, .maxSpeed = 58});
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(250);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, -47, 700, {.forwards = false, .minSpeed = 35});
    chassis.waitUntilDone();
    toggle_score(true, 127, 127);
    matchLoad.set_value(false);
    chassis.tank(-40, -40, true);
    score.set_value(true);
    pros::delay(1200);
    chassis.setPose(-29.5, -47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    toggle_score(false);
    score.set_value(false);
    goalPush(1);
    chassis.tank(40, 45, true);
    pros::delay(400);
    chassis.tank(0, 0, true);
    chassis.turnTo(155, 500, {.minSpeed = 20, .earlyExitRange = 3});
    antenne.set_value(true);
    toggle_preroller(true);
    chassis.moveTo(-7.5, -46, 800, {.maxSpeed = 70, .minSpeed = 30});
    matchLoad.set_value(false);
    chassis.waitUntil(3);
    matchLoad.set_value(true);
    chassis.moveTo(-16, -38, 900, {.forwards = false, .maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 3});
    chassis.turnTo(45, 500, {.minSpeed = 20, .earlyExitRange = 3});
    matchLoad.set_value(false);
    chassis.moveTo(-7.6, -7.2, 45, 1500, {.maxSpeed = 100, .minSpeed = 10});
    chassis.waitUntil(22);
    toggle_score(true, -70, -80);
    chassis.waitUntilDone();
    for (int i = 0; i < 7; i++) {
        chassis.tank(i % 2 == 0 ? 40 : -40, i % 2 == 0 ? -40 : 40, true);
        pros::delay(150);
    }
    antenne.set_value(false);
    chassis.moveTo(-29, -32, 1000, {.forwards = false});
    chassis.moveTo(-13, -36.4, 270, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 20});
    score.set_value(false);

}

// Right 9 Split Goal
void auton6() {
    //RIGHT + LOWER
    chassis.setPose(-46, -14.5, 90);
    antenne.set_value(true);
    chassis.moveTo(-27, -20, 1000, {.minSpeed = 20});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    toggle_preroller(true);
    chassis.waitUntil(15);
    matchLoad.set_value(true);
    chassis.moveTo(-7.5, -46, 800, {.maxSpeed = 70, .minSpeed = 30});
    matchLoad.set_value(false);
    chassis.waitUntil(17);
    matchLoad.set_value(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-37, -39, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 3});
    chassis.moveTo(-35, -48, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 3});
    toggle_preroller(false);
    //chassis.turnTo(270, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    //chassis.resetOdometry(30, true);
    //pros::delay(50);
    chassis.moveTo(-22, -47, 270, 1000, {.forwards = false, .lead = 0});
    chassis.turnTo(270, 200, {.minSpeed = 40, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.tank(-40, -40, true);
    toggle_score(true, 127, 127);
    score.set_value(true);
    pros::delay(1200);
    chassis.setPose(-29.25, -47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    toggle_score(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-78, -47, 800, {.minSpeed = 20, .earlyExitRange = 40});
    score.set_value(false);
    chassis.moveTo(-78, -47, 270, 800, {.lead = 0, .maxSpeed = 53});
    toggle_preroller(true);
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(200);
    chassis.tank(0, 0, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-35, -30, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
    chassis.waitUntil(8);
    matchLoad.set_value(false);
    chassis.turnTo(45, 500, {.minSpeed = 10, .earlyExitRange = 3});
    chassis.moveTo(-14, -11, 45, 1500, {.maxSpeed = 100, .minSpeed = 10});
    chassis.turnTo(45, 500, {.minSpeed = 10, .earlyExitRange = 3});
    chassis.waitUntilDone();
    toggle_score(true, -70, -80);
    pros::delay(1000);
    /*
    for (int i = 0; i < 7; i++) {
        chassis.tank(i % 2 == 0 ? 40 : -40, i % 2 == 0 ? -40 : 40, true);
        pros::delay(150);
    }
        */
    antenne.set_value(false);
    chassis.moveTo(-29, -33, 1000, {.forwards = false});
    chassis.moveTo(-7, -35.5, 270, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 20});
    score.set_value(false);
    chassis.turnTo(235, 500);
}

// Fast Left 4
void auton7() {
    chassis.setPose(-46, 14.5, 90);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    chassis.moveTo(-26.8, 21, 1000, {.minSpeed = 30});
    toggle_preroller(true);
    chassis.waitUntil(13);
    matchLoad.set_value(true);
    pros::delay(100);
    fourBallArc(1);
    toggle_score(true, 120, 120);
    score.set_value(true);
    pros::delay(800);
    chassis.setPose(-29.5, 47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    toggle_score(false);
    score.set_value(false);
    goalPush(-1);
}

// Left 7
void auton8() {
    // Left 7
    chassis.setPose(-46, 14.5, 90);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    //chassis.turnTo(-22, -22.5, 500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveTo(-26.8, 21, 1000, {.minSpeed = 30});
    toggle_preroller(true);
    chassis.waitUntil(15);
    matchLoad.set_value(true);
    pros::delay(100);
    //chassis.turnTo(250, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-45, 43, 2000, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.moveTo(-75, 48, 270, 700, {.lead = 0, .maxSpeed = 58});
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(350);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, 47, 800, {.forwards = false, .minSpeed = 35});
    chassis.waitUntil(30);
    toggle_score(true, -40, -70);
    chassis.waitUntilDone();
    toggle_score(true, 127, 127);
    chassis.tank(-40, -40, true);
    matchLoad.set_value(false);
    score.set_value(true);
    pros::delay(1200);
    chassis.setPose(-29.5, 47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    toggle_score(false);
    score.set_value(false);
    goalPush(-1);
}

// Left 7 Split
void auton9() {
    // LEFT 7 Split
    chassis.setPose(-49, 18, 180);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    pros::delay(0);
    chassis.moveTo(-46, 39, 800, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
    chassis.turnTo(270, 500, {.minSpeed = 40, .earlyExitRange = 3});
    matchLoad.set_value(true);
    toggle_preroller(true);
    chassis.moveTo(-78, 47, 270, 900, {.lead = 0, .maxSpeed = 58, .minSpeed = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(150);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, 47, 270, 900, {.forwards = false, .maxSpeed = 100, .minSpeed = 20});
    chassis.waitUntil(60);
    toggle_score(true, 120, 120);
    score.set_value(true);
    chassis.tank(-25, -25, true);
    pros::delay(800);
    chassis.setPose(-29.5, 47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    toggle_score(false);

    toggle_preroller(true);
    chassis.moveTo(-37, 47, 500, {.minSpeed = 40, .earlyExitRange = 5});
    score.set_value(false);
    chassis.turnTo(175, 800, {.minSpeed = 20, .earlyExitRange = 3});
    toggle_preroller(true);
    chassis.moveTo(-13, 13, 500, {.minSpeed = 10});
    chassis.waitUntilDone();
    matchLoad.set_value(true);
    pros::delay(200);
    chassis.turnTo(315, 500, {.minSpeed = 35, .earlyExitRange = 3});
    chassis.moveTo(-6.6, 8.6, 315, 1000, {.forwards = false, .maxSpeed = 85, .minSpeed = 10});
    chassis.waitUntil(10);
    toggle_score(true, -50, -80);
    chassis.waitUntilDone();
    chassis.tank(-25, -25, true);
    centerGoal.set_value(true);
    toggle_score(true, 100, -100);
    pros::delay(1000);
    chassis.tank(70, 70, true);
    antenne.set_value(false);
    matchLoad.set_value(false);
    descore.set_value(true);
    pros::delay(400);
    centerGoal.set_value(false);
    toggle_score(false);
    chassis.tank(-60, -60, true);
    pros::delay(500);
    chassis.tank(-20, -20, true);
    for (int i = 0; i < 10; i++){
        chassis.tank(35, 35, true);
        pros::delay(250);
        chassis.tank(-40, -40, true);
        pros::delay(260);
    }
    descore.set_value(false);
    
}

// Left 7 Center First
void auton10() {
    // Left 7 Center First
    chassis.setPose(-46, 14.5, 90);
    antenne.set_value(true);
    pros::delay(0);
    chassis.moveTo(-25, 22, 1000, {.minSpeed = 20});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    toggle_preroller(true, 110);
    chassis.waitUntil(13);
    matchLoad.set_value(true);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveTo(-6.5, 5.5, 315, 900, {.forwards = false, .lead = 0, .maxSpeed = 60, .minSpeed = 0});
    pros::delay(400);
    toggle_score(true, -50, -80);
    pros::delay(250);
    toggle_score(false);
    centerGoal.set_value(true);
    chassis.waitUntilDone();
    chassis.tank(-25, -25, true);
    pros::delay(150);
    toggle_score(true, 100, -100);
    pros::delay(800);
    chassis.tank(0, 0, true);
    chassis.moveTo(-47, 51, 800, {.minSpeed = 10, .earlyExitRange = 3});
    chassis.waitUntil(2);
    toggle_score(false);
    centerGoal.set_value(false);
    toggle_preroller(true);
    //chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    toggle_preroller(true);
    reset.updateBotPose(&right_rcl);
    pros::delay(50);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-85, 46.5, 270, 800, {.lead = 0, .maxSpeed = 60, .minSpeed = 10});
    chassis.waitUntilDone();
    chassis.tank(60, 60, true);
    pros::delay(1000);
    chassis.tank(0, 0, true);
    chassis.moveTo(-13, 47, 270, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 10});
    chassis.waitUntilDone();
    chassis.tank(-40, -40, true);
    score.set_value(true);
    toggle_score(true, 115, 115);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(700);
    toggle_score(false);
    chassis.setPose(-29.5, 47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    score.set_value(false);
    matchLoad.set_value(false);
    goalPush(-1);

}



/*
// Left 4 NOT USING
void auton6() {
    // LEFT 4
    chassis.setPose(-49, 18, 180);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    pros::delay(0);
    chassis.moveTo(-46, 39, 800, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
    chassis.turnTo(270, 500, {.minSpeed = 40, .earlyExitRange = 5});
    matchLoad.set_value(true);
    toggle_preroller(true);
    chassis.moveTo(-78, 47, 270, 900, {.lead = 0, .maxSpeed = 58, .minSpeed = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(280);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, 47, 270, 900, {.forwards = false, .maxSpeed = 100, .minSpeed = 20});
    chassis.waitUntil(60);
    toggle_score(true, 120, 120);
    score.set_value(true);
    chassis.tank(-30, -30, true);
    pros::delay(800);
    chassis.setPose(-29.5, 47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    toggle_score(false);
    score.set_value(false);
    goalPush(-1);
}

// Right 4 NOT USING
void auton10() {
    // RIGHT 4
    chassis.setPose(-49, -18, 0);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    pros::delay(0);
    chassis.moveTo(-46, -39, 800, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
    chassis.turnTo(270, 500, {.minSpeed = 40, .earlyExitRange = 5});
    matchLoad.set_value(true);
    toggle_preroller(true);
    chassis.moveTo(-78, -47, 270, 900, {.lead = 0, .maxSpeed = 58, .minSpeed = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(280);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, -47, 270, 900, {.forwards = false, .maxSpeed = 100, .minSpeed = 20});
    chassis.waitUntil(60);
    toggle_score(true, 120, 120);
    score.set_value(true);
    chassis.tank(-30, -30, true);
    pros::delay(800);
    chassis.setPose(-29.5, -47, chassis.getPose().theta);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    toggle_score(false);
    score.set_value(false);
    goalPush(1);
}

// Normal Sawp NOT USING
void auton2() {
    // NORMAL SAWP
    chassis.setPose(-46, 0.5, 180);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    pros::delay(0);
    chassis.tank(-50, -50, true);
    pros::delay(250);
    chassis.tank(0, 0, true);
    chassis.moveTo(-46, -33, 2000, {.maxSpeed = 100, .minSpeed = 15, .earlyExitRange = 3});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(5);
    matchLoad.set_value(true);
    chassis.waitUntilDone();
    //chassis.resetOdometry();
    chassis.turnTo(270, 800, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    reset.updateBotPose(&left_rcl);
    pros::delay(50);
    chassis.moveTo(-78, -47, 270, 900, {.lead = 0, .maxSpeed = 55});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    reset.updateBotPose(&left_rcl);
    pros::delay(350);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, -47, 270, 900, {.forwards = false, .maxSpeed = 100, .minSpeed = 20});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(8);
    toggle_score(true, 80, 10);
    chassis.waitUntil(38);
    score.set_value(true);
    toggle_score(true, 127, 127);
    chassis.waitUntilDone();
    matchLoad.set_value(false);
    chassis.tank(-35, -35, true);
    pros::delay(800);
    chassis.tank(0, 0, true);
    chassis.setPose(-29.5, -47, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE chassis.getPose().theta
    toggle_score(false);

    toggle_preroller(true);
    chassis.moveTo(-37, -47, 500, {.minSpeed = 40, .earlyExitRange = 5});
    score.set_value(false);
    chassis.turnTo(5, 800, {.minSpeed = 20, .earlyExitRange = 3});
    toggle_score(true, 80, 80);
    chassis.moveTo(-16.5, -15.5, 500, {.minSpeed = 10});
    toggle_score(false);
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(250);
    matchLoad.set_value(true);
    chassis.turnTo(0, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-22, 37.5, 1000, {.maxSpeed = 70, .minSpeed = 10}); //max = 70
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    pros::delay(130);
    matchLoad.set_value(true);
    pros::delay(0);
    chassis.turnTo(315, 500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveTo(-10.6, 6.6, 315, 1300, {.forwards = false, .lead = 0, .maxSpeed = 75, .minSpeed = 20});
    chassis.waitUntil(14);
    toggle_score(true, -25, -50);
    chassis.waitUntil(27.5);
    centerGoal.set_value(true);
    chassis.tank(-35, -35, true);
    toggle_score(true, 100, -120);
    pros::delay(800);
    chassis.moveTo(-57, 46, 800, {.minSpeed = 10, .earlyExitRange = 5});
    chassis.waitUntil(1);
    
    toggle_score(false);
    centerGoal.set_value(false);
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    reset.updateBotPose(&right_rcl);
    pros::delay(50);
    //matchLoad.set_value(false); // REMOVE LATER
    chassis.moveTo(-85, 47, 270, 1200, {.maxSpeed = 58, .minSpeed = 10});
    toggle_preroller(true);
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(500);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, 47, 270, 1100, {.forwards = false, .minSpeed = 15});
    chassis.waitUntil(8);
    toggle_score(true, 80, 10);
    chassis.waitUntil(40);
    score.set_value(true);
    toggle_score(true, 127, 127);
    chassis.waitUntilDone();
    chassis.tank(-30, -30, true);
    matchLoad.set_value(false);
    
}
*/