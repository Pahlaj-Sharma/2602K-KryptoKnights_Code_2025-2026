#include "main.h"
#include "pahlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"

using namespace pahlib;

ASSET(path_txt);

void auton1() {
    chassis.setPose(0, 0, 0);
    chassis.setPID(Chassis::PIDPreset::normal);
    chassis.setPID(2.0, 0.0, 16.0, 0.0, 2.0, 0.0, 16.0, 0.0); // Custom PID values for this auton
    chassis.moveLinear(12);
    chassis.resetOdometry();
    chassis.follow(path_txt, 2, 10000);
    chassis.moveToPose(14, 14, 90, 3000, {}, {{2, 0, 16}}, {{2, 0, 16}});
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
