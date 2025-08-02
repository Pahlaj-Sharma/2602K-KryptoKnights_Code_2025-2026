#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP
#include "main.h"
#include "lemlib/api.hpp"
#include <string>

enum class PIDPreset {
    normal,
    fast,
    precise
};

void moveLinear(double inches, int timeout = 2000, float lead = 0.2, float maxspeed = 70, float minspeed = 40);

// You can create an overloaded version to handle your 'premade' presets
void chassisPID(PIDPreset premade);
// This version allows the caller to set all PID constants.
void chassisPID(double lat_kp, double lat_ki, double lat_kd, double ang_kp, double ang_ki, double ang_kd);

void resetOdometry(double threshold = 5.0);

#endif