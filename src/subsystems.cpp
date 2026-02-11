#include "main.h"
#include "robot_config.hpp"
#include "subsystems.hpp"

void toggle_preroller(bool toggled, int vel) {
    intake_motor.move(-vel * toggled * antiJam);
}

void toggle_score(bool toggled, int vel, int topVel) {
    score_motor.move(topVel * toggled * antiJam);
    intake_motor.move(-vel * toggled * antiJam);
}