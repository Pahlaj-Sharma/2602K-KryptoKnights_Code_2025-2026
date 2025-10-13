#include "main.h"
#include "pahlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"

using namespace pahlib;

ASSET(path_txt);

void auton1() {
    chassis.setPose(0, 0, 0);
    chassis.moveTo(10, 10, 45, 1000, {.gainScheduling = false});
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
