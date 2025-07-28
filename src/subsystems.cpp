#include "main.h"
#include "lemlib/api.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"


// Code for subsytems goes here
void toggle_pto(bool state){
    pto.set_value(state); // Set the PTO output based on the new state
    if (ptoState) {
        left_motors.append(left_PTO); right_motors.append(right_PTO);
    } else {
        left_motors.erase_port(PORT_LEFT_PTO); right_motors.erase_port(PORT_RIGHT_PTO);
    }
}

