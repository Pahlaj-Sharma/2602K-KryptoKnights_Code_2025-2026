#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "functions.hpp"

const asset path_jerryio_txt {};

void auton1() {
    if (teamType == "RED") {
        chassis.setPose(0, 0, 0);
        chassisPID(PIDPreset::normal);
        chassisPID(2.0, 0.0, 16.0, 2.0, 0.0, 16.0); // Custom PID values for this auton
        moveLinear(12);
        resetOdometry();
        chassis.follow(path_jerryio_txt, 2, 10000);
        chassis.moveToPose(14, 14, 90, 3000, {}, {{2, 0, 16}}, {{2, 0, 16}});
    } else {
        chassis.setPose(0, 0, 0);
        chassisPID(PIDPreset::normal);
        chassisPID(2.0, 0.0, 16.0, 2.0, 0.0, 16.0); // Custom PID values for this auton
        moveLinear(12);
        resetOdometry();
    }
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
