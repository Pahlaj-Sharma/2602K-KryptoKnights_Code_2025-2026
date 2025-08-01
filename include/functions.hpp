#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP
#include "main.h"
#include "lemlib/api.hpp"
#include <string>

void moveLinear(double inches, int timeout = 2000, float lead = 0.2, float maxspeed = 70, float minspeed = 40);
void chassisPID(std::string premade = "normal", double lat_kp = chassis.lateralPID.kP, double lat_ki = chassis.lateralPID.kI, double lat_kd = chassis.lateralPID.kD, double ang_kp = chassis.angularPID.kP, double ang_ki = chassis.angularPID.kI, double ang_kd = chassis.angularPID.kD);
void resetOdometry(int threshold = 5);

#endif