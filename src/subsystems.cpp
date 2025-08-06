#include "main.h"
#include "lemlib/api.hpp"
#include "robot_config.hpp"
#include "subsystems.hpp"

void toggle_pto(bool state) {
    // Update the global state variable
    ptoState = state;

    // Set the digital output to control the physical PTO
    pto.set_value(state);

    // Reconfigure the motor groups based on the new PTO state
    if (ptoState) {
        // PTO is engaged for intake, add the PTO motors to the drivetrain groups
        left_motors.append(left_pto); right_motors.append(right_pto);
    } else {
        // PTO is disengaged for drivetrain, remove the PTO motors from the groups
        left_motors.erase_port(PORT_LEFT_PTO); right_motors.erase_port(PORT_RIGHT_PTO);
    }
}