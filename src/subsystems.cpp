#include "main.h"
#include "robot_config.hpp"
#include "subsystems.hpp"

void toggle_preroller(bool toggled, int vel) {
    intake_motor.move(-vel * toggled);
}

void toggle_score(bool toggled, int vel, int topVel, bool disableAntiJam) {
    score_motor.move(topVel * toggled);
    intake_motor.move(-vel * toggled);
    antiJamEnable = !disableAntiJam;
}