#include "main.h"
#include "pahlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"

using namespace pahlib;

ASSET(path_txt);

void auton1() {
    chassis.setPose(-47.75, -13.75, 90);
    chassis.moveTo(-18, -26, 110, 2000, {.lead = 0.8, .minSpeed = 20, .earlyExitRange = 8});
    chassis.turnTo(-8, -42, 500);
    chassis.moveTo(-8, -42, 150, 2000, {.lead = 0.8});
    chassis.moveTo(-30, -30, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});
    chassis.swingTo(270, DriveSide::RIGHT, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 20});
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
