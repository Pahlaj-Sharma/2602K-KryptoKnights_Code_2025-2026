#include "main.h"
#include "pahlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"

using namespace pahlib;

void matchLoadMove(int amt = 3){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    for (int i = 0; i < amt; i++) {
        chassis.tank(75, 75, true);
        pros::delay(200);
        chassis.tank(-5, -5, true);
        pros::delay(200);
    }
}

/*
    Better way to set position
    x = TILE_LENGTH (140.25 / 6) * NUM_TILES_FROM_CENTER * DIRECTION_SIGN + OFFSET - DRIVE_WIDTH or DRIVE_LENGTH based on angle
    y = TILE_LENGTH (140.25 / 6) * NUM_TILES_FROM_CENTER * DIRECTION_SIGN + OFFSET - DRIVE_WIDTH or DRIVE_LENGTH based on angle
    // 23.54944882 is field tile length

    DISTANCE RESET
    reset.updateBotPose(&left_rcl);   // Distance reset on the left sensor
    reset.updateBotPose();    // Update chassis position based on RCL
    reset.setRclPose(chassis.getPose());  // Reset Rcl Pose to Lemlib Pose

*/

void auton1() {
    // SKILLS
    /*
    chassis.setPose(-46, 14.5, 90);
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);
    chassis.moveTo(-27, 21.5, 63, 1000, {.minSpeed = 5}); // 61, remove lead
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(14);
    toggle_preroller(true, 120);
    matchLoad.set_value(true);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveTo(-27.5, 22, 500);
    chassis.turnTo(315, 500);
    chassis.moveTo(-9, 6.5, 315, 1000, {.forwards = false});
    chassis.waitUntil(10);
    toggle_preroller(false);
    toggle_score(true, -10, -50);
    chassis.waitUntilDone();
    centerGoal.set_value(true);
    chassis.tank(-35, -35, true);
    toggle_score(true, 100, -120);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(550);
    chassis.tank(0, 0, true);
    chassis.moveTo(-47, 47, 315, 1000);
    chassis.waitUntil(1);
    toggle_score(false);
    centerGoal.set_value(false);
    toggle_preroller(true);
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    reset.updateBotPose(&right_rcl);
    pros::delay(50);
    chassis.moveTo(-10, 46, 270, 800, {.forwards = false, .minSpeed = 20});
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
    matchLoadMove(3);
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
    matchLoadMove(3);
    toggle_preroller(true, 127);
    pros::delay(200);
    chassis.moveTo(15, 47, 90, 1000, {.forwards = false});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    toggle_preroller(true, -10);
    chassis.waitUntil(15);
    toggle_preroller(false);
    chassis.waitUntilDone();
    chassis.tank(-30, -30, true);
    chassis.turnTo(90, 500, {.minSpeed = 40});
    score.set_value(true);
    toggle_score(true, 127, 127);
    pros::delay(1500);
    chassis.tank(0, 0, true);
    chassis.setPose(29.5, 47, chassis.getPose().theta);
    score.set_value(false);
    chassis.tank(35, 35, true);
    pros::delay(350);
    chassis.tank(-25, -25, true);
    pros::delay(550);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    toggle_score(false);
    pros::delay(50);
*/
    chassis.setPose(29.5, 47, 90); //deltete
    reset.setRclPose(chassis.getPose());
    antenne.set_value(true);

    toggle_score(true, 70, 70);
    chassis.tank(50, 50, true);
    pros::delay(300);
    chassis.tank(0, 0, true);
    chassis.moveTo(60, 30, 900, {.maxSpeed = 80});
    chassis.turnTo(180, 500, {.maxSpeed = 100});
    chassis.moveTo(62.5, 16, 900, {.maxSpeed = 70});
    chassis.turnTo(175, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    
    doublePark.set_value(true);
    toggle_score(false);
    chassis.tank(-40, -40, true);
    pros::delay(200);
    chassis.tank(65, 65, true);
    pros::delay(200);
    chassis.tank(75, 85, true);
    toggle_preroller(true, 115);
    pros::delay(1500);
    matchLoad.set_value(true);
    chassis.tank(60, 70, true);
    pros::delay(500);
    chassis.tank(0, 0, true);
    doublePark.set_value(false);
    chassis.setPose(65, -20, chassis.getPose().theta);
    chassis.turnTo(180, 500);
    chassis.waitUntilDone();
    chassis.tank(-40, -50, true);
    pros::delay(800);
    chassis.tank(0, 0, true);
    chassis.setPose(chassis.getPose().x, -16.25, chassis.getPose().theta);
    reset.setRclPose(chassis.getPose());
    pros::delay(100);
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
    toggle_preroller(true);
    chassis.moveTo(26.45, -21.2, 225, 1000);
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    chassis.tank(-50, -10, true);
    pros::delay(350);
    chassis.tank(0, 0, true);
    //matchLoad.set_value(true);
    chassis.turnTo(135, 500);
    chassis.moveTo(10.7, -6.2, 135, 1000, {.forwards = false, .lead = 0, .maxSpeed = 60});
    toggle_preroller(false);
    toggle_score(true, -15, -80);
    pros::delay(400);
    matchLoad.set_value(true);
    toggle_score(false);
    centerGoal.set_value(true);
    chassis.turnTo(135, 500, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.tank(15, -120, true);
    pros::delay(200);
    toggle_score(true, 90, -120);
    pros::delay(200);
    chassis.tank(-35, -35, true);
    chassis.turnTo(135, 500);
    pros::delay(700);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    for (int i = 0; i < 2; i++) {
        chassis.tank(-40, -40, true);
        pros::delay(200);
        chassis.tank(35, 35, true);
        pros::delay(200);
    }
    for (int i = 0; i < 2; i++) {
        chassis.tank(-40, -40, true);
        pros::delay(200);
        chassis.tank(5, 5, true);
        pros::delay(200);
    }
    chassis.tank(-60, -60, true);
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
    pros::delay(400);
    chassis.tank(0, 0, true);

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
    matchLoadMove(4);
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
    chassis.moveTo(-15, -47, 90, 1000, {.forwards = false});
    toggle_preroller(true);
    chassis.waitUntil(20);
    toggle_score(-20, -30);
    chassis.turnTo(270, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    chassis.tank(-30, -30, true);
    toggle_score(true, 127, 127);
    score.set_value(true);
    pros::delay(1400);
    chassis.tank(0, 0, true);
    score.set_value(false);
    toggle_score(false);
    toggle_preroller(true);
    chassis.setPose(-29.5, -47, chassis.getPose().theta);
    chassis.moveTo(-70, -47, 1500, {.minSpeed = 10, .earlyExitRange = 35});
    chassis.moveTo(-70, -47, 270, 800, {.maxSpeed = 53});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    matchLoadMove(4);
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
    score.set_value(false);
    chassis.tank(50, 50, true);
    pros::delay(350);
    chassis.tank(-50, -50, true);
    pros::delay(450);
    chassis.tank(0, 0, true);
    matchLoad.set_value(false);
    score.set_value(true);
    toggle_score(true, 70, 90);
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
    toggle_score(true, 115, 115);
    pros::delay(1000);
    chassis.tank(0, 0, true);
    score.set_value(false);
}

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
    chassis.moveTo(-78, -45, 270, 900, {.lead = 0, .maxSpeed = 54});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    chassis.tank(40, 40, true);
    pros::delay(300);
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
    pros::delay(300);
    matchLoad.set_value(true);
    pros::delay(0);
    chassis.moveTo(-8.5, 7, 315, 1300, {.forwards = false, .maxSpeed = 75, .minSpeed = 20});
    chassis.waitUntil(14);
    toggle_score(true, -25, -50);
    chassis.waitUntil(27.5);
    centerGoal.set_value(true);
    chassis.tank(-35, -35, true);
    toggle_score(true, 100, -120);
    pros::delay(800);
    chassis.moveTo(-42, 45, 800, {.minSpeed = 10, .earlyExitRange = 5});
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
    pros::delay(400);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, 47, 270, 1100, {.forwards = false, .minSpeed = 15});
    chassis.waitUntil(8);
    toggle_score(true, 80, 10);
    chassis.waitUntil(40);
    score.set_value(true);
    toggle_score(true, 127, 127);
    chassis.waitUntilDone();
    matchLoad.set_value(false);
    
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
    pros::delay(100);
    chassis.turnTo(250, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-42, -40, 2000, {.minSpeed = 50, .earlyExitRange = 2});
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-75, -47, 270, 900, {.lead = 0, .maxSpeed = 58});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveTo(-15, -47, 270, 1200, {.forwards = false});
    chassis.waitUntilDone();
    toggle_score(true, 127, 127);
    chassis.turnTo(270, 500);
    pros::delay(100);
    matchLoad.set_value(false);
    score.set_value(true);
    pros::delay(1500);
    chassis.setPose(-29.5, -47, chassis.getPose().theta);
    toggle_score(false);
    score.set_value(false);
    chassis.moveTo(-38, -36, 500, {.maxSpeed = 90});
    matchLoad.set_value(false);
    chassis.moveTo(-13, -36.5, 270, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
    chassis.waitUntil(5);
    antenne.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnTo(245, 350, {.minSpeed = 50});
}

void auton4() {
    // LEFT 9 split
    chassis.setPose(-46, 14.5, 90);
    antenne.set_value(true);
    pros::delay(0);
    chassis.turnTo(-22, 22.5, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-27, 21, 1000, {.minSpeed = 20});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    toggle_preroller(true);
    chassis.waitUntil(15);
    matchLoad.set_value(true);
    chassis.turnTo(-10.75, 45, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-10.5, 46, 800, {.maxSpeed = 70, .minSpeed = 30});
    matchLoad.set_value(false);
    chassis.waitUntil(17.5);
    matchLoad.set_value(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(100);
    chassis.moveTo(-19.4, 24.5, 800, {.forwards = false, .maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 3});
    chassis.moveTo(-5.4, 5.4, 315, 1200, {.forwards = false, .maxSpeed = 90, .minSpeed = 5});
    toggle_score(true, -40, -80);
    pros::delay(200);
    toggle_score(false);
    chassis.waitUntilDone();
    chassis.turnTo(315, 500, {.minSpeed = 20});
    chassis.tank(-55, -55, true);
    centerGoal.set_value(true);
    toggle_score(true, 80, -100);
    pros::delay(700);
    chassis.tank(0, 0, true);
    chassis.moveTo(-47, 47, 800, {.minSpeed = 5, .earlyExitRange = 3});
    chassis.waitUntil(2);
    toggle_score(false);
    centerGoal.set_value(false);
    toggle_preroller(true);
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    toggle_preroller(true);
    reset.updateBotPose(&right_rcl);
    reset.updateBotPose(&front_rcl);
    pros::delay(50);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-85, 47, 270, 700, {.lead = 0, .maxSpeed = 60, .minSpeed = 10});
    chassis.waitUntilDone();
    chassis.tank(60, 60, true);
    pros::delay(800);
    chassis.tank(0, 0, true);
    chassis.moveTo(-15, 47, 270, 800, {.forwards = false, .maxSpeed = 70, .minSpeed = 10});
    chassis.waitUntilDone();
    score.set_value(true);
    toggle_score(true, 115, 115);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(1000);
    toggle_score(false);
    chassis.setPose(-29.5, 47, chassis.getPose().theta);
    score.set_value(false);
    matchLoad.set_value(false);
    chassis.moveTo(-38, 58, 500, {.maxSpeed = 90});
    matchLoad.set_value(false);
    chassis.moveTo(-13, 57.5, 270, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
    chassis.waitUntil(5);
    antenne.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnTo(245, 350, {.minSpeed = 50});

    /*
    chassis.moveTo(-37, 39, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 3});
    chassis.moveTo(-38, 49, 1000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 3});
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-22, 51, 270, 1000, {.forwards = false, .lead = 0.3});
    chassis.waitUntilDone();
    chassis.turnTo(270, 500);
    toggle_score(true, 127, 127);
    score.set_value(true);
    pros::delay(1200);
    chassis.setPose(-29.25, 47, chassis.getPose().theta);
    toggle_score(false);
    score.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-78, 46, 270, 1500, {.lead = 0, .maxSpeed = 53});
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(150);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-43, 30, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
    chassis.turnTo(315, 500, {.minSpeed = 10, .earlyExitRange = 3});
    chassis.moveTo(-6.7, 6, 315, 1300, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(14);
    toggle_score(true, -25, -50);
    chassis.waitUntil(27.5);
    centerGoal.set_value(true);
    toggle_score(true, 100, -100);
    matchLoad.set_value(false);
    pros::delay(1000);
    centerGoal.set_value(false);
    antenne.set_value(false);
    chassis.moveTo(-28, 32, 1000);
    chassis.turnTo(90, 500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-6, 35.5, 90, 1000, {.maxSpeed = 100});
    chassis.turnTo(135, 500);
    */
}

void auton5() {
    //RIGHT + LOWER
    chassis.setPose(-46, -14.5, 90);
    antenne.set_value(true);
    chassis.turnTo(-22, -22.5, 500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveTo(-27, -22, 1000, {.minSpeed = 20});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    toggle_preroller(true);
    chassis.waitUntil(15);
    matchLoad.set_value(true);
    chassis.turnTo(-10, -46, 500, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-12, -46, 800, {.maxSpeed = 70, .minSpeed = 30});
    matchLoad.set_value(false);
    chassis.waitUntil(17);
    matchLoad.set_value(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-37, -39, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 3});
    chassis.moveTo(-38, -49, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 3});
    chassis.turnTo(270, 500, {.minSpeed = 20});
    chassis.waitUntilDone();
    //chassis.resetOdometry(30, true);
    pros::delay(50);
    chassis.moveTo(-22, -55, 270, 1000, {.forwards = false, .lead = 0.3});
    chassis.waitUntilDone();
    chassis.turnTo(270, 500);
    toggle_score(true, 127, 127);
    score.set_value(true);
    pros::delay(1200);
    chassis.setPose(-29.25, -47, chassis.getPose().theta);
    toggle_score(false);
    score.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-78, -46, 270, 1500, {.lead = 0, .maxSpeed = 53});
    toggle_preroller(true);
    chassis.waitUntilDone();
    pros::delay(290);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveTo(-43, -30, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
    chassis.turnTo(45, 500, {.minSpeed = 10, .earlyExitRange = 3});
    chassis.moveTo(-11.5, -12, 45, 1500, {.maxSpeed = 90});
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    toggle_score(true, -80, -80);
    score.set_value(true);
    pros::delay(1000);
    antenne.set_value(false);
    chassis.moveTo(-29, -33.5, 1000, {.forwards = false});
    chassis.moveTo(-8, -36, 270, 1000, {.forwards = false, .maxSpeed = 100});
    score.set_value(false);
    chassis.turnTo(235, 500);
}

void auton6() {
    // LEFT 4
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
    pros::delay(300);
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
    chassis.moveTo(-38, 58, 500, {.maxSpeed = 90});
    matchLoad.set_value(false);
    chassis.moveTo(-13, 57.5, 270, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
    chassis.waitUntil(5);
    antenne.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnTo(245, 350, {.minSpeed = 50});
}

void auton7() {
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
    pros::delay(200);
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

    toggle_preroller(true);
    chassis.moveTo(-37, 47, 500, {.minSpeed = 40, .earlyExitRange = 5});
    score.set_value(false);
    chassis.turnTo(175, 800, {.minSpeed = 20, .earlyExitRange = 3});
    toggle_preroller(true);
    chassis.moveTo(-15.5, 14.5, 500, {.minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(250);
    matchLoad.set_value(true);
    pros::delay(50);
    chassis.turnTo(315, 500, {.minSpeed = 35, .earlyExitRange = 3});
    chassis.moveTo(-6.4, 8.6, 315, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 10});
    chassis.waitUntil(10);
    toggle_score(true, -50, -80);
    chassis.waitUntilDone();
    chassis.tank(-30, -30, true);
    centerGoal.set_value(true);
    toggle_score(true, 100, -100);
    pros::delay(1000);
    chassis.tank(0, 0, true);
    chassis.moveTo(-28, 32, 1000, {.minSpeed = 30, .earlyExitRange = 2});
    antenne.set_value(false);
    chassis.waitUntil(1);
    matchLoad.set_value(false);
    centerGoal.set_value(false);
    toggle_score(false);
    chassis.turnTo(90, 500, {.minSpeed = 20, .earlyExitRange = 2});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-6, 35.5, 90, 1000, {.minSpeed = 5});
    chassis.turnTo(135, 500);
    
}

void auton8() {
    
}

void auton9() {
    
}

void auton10() {
    
}

/*
// COUNTER SAWP
    chassis.setPose(-46, 0.5, 180);
    antenne.set_value(true);
    pros::delay(0);
    chassis.moveTo(-46, -37, 2000, {.maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 2});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(5);
    matchLoad.set_value(true);
    chassis.waitUntilDone();
    //chassis.resetOdometry();
    pros::delay(50);
    chassis.turnTo(270, 800, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-78, -44.5, 270, 900, {.lead = 0, .maxSpeed = 57});
    toggle_preroller(true, 100);
    chassis.waitUntilDone();
    pros::delay(350);
    chassis.moveTo(-27, -44.5, 270, 1000, {.forwards = false});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntil(8);
    toggle_score(true, 80, 10);
    chassis.waitUntil(35);
    toggle_score(true, 127, 127);
    chassis.waitUntilDone();
    matchLoad.set_value(false);
    score.set_value(true);
    pros::delay(800);
    chassis.setPose(-29.25, -47, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE chassis.getPose().theta
    toggle_score(false);
    toggle_preroller(true);
    chassis.moveTo(-37, -47, 500, {.minSpeed = 40, .earlyExitRange = 5});
    score.set_value(false);
    chassis.turnTo(5, 800, {.minSpeed = 20, .earlyExitRange = 3});
    toggle_preroller(true);
    chassis.moveTo(-16.5, -15.5, 500);
    chassis.waitUntilDone();
    pros::delay(250);
    matchLoad.set_value(true);
    chassis.turnTo(0, 500, {.minSpeed = 35, .earlyExitRange = 3});
    chassis.moveTo(-22, 28, 1000, {.maxSpeed = 70});
    matchLoad.set_value(false);
    chassis.waitUntilDone();
    pros::delay(150);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    matchLoad.set_value(true);
    pros::delay(0);
    chassis.moveTo(-44, 40, 1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnTo(270, 500, {.minSpeed = 20, .earlyExitRange = 3});
    //chassis.resetOdometry();
    pros::delay(50);
    chassis.moveTo(-23, 44.1, 270, 1400, {.forwards = false});
    chassis.waitUntilDone();
    score.set_value(true);
    toggle_score(true, 127, 127);
    pros::delay(1000);
    chassis.setPose(-29.25, 47, chassis.getPose().theta); // CHAGE THETA TO THE CHASSIS VALUE chassis.getPose().theta
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveTo(-78, 47, 270, 1400, {.maxSpeed = 53});
    toggle_score(false);
    toggle_preroller(true);
    score.set_value(false);
    chassis.waitUntilDone();
    //chassis.resetOdometry();
    pros::delay(400);
    chassis.moveTo(chassis.getPose().x + 8, chassis.getPose().y - 2, 500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 3});
    chassis.moveTo(-5, 9, 315, 1500, {.forwards = false, .minSpeed = 20});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(15);
    toggle_score(true, -40, -80);
    pros::delay(300);
    toggle_score(false);
    chassis.waitUntilDone();
    centerGoal.set_value(true);
    toggle_score(true, 80, -100);
    matchLoad.set_value(false);

*/