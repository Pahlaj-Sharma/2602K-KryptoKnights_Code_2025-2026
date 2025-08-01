#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "robot_config.hpp"
#include "functions.hpp"
#include <cmath>
#include <tuple>
#include <vector>

ASSET(path_jerryio_txt);

void auton1() {
    chassis.setPose(0, 0, 0);
    moveLinear(12);
    chassisPID("precise");
    resetOdometry();
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
