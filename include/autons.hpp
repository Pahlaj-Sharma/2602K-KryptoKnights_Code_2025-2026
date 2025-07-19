#ifndef AUTONS_HPP
#define AUTONS_HPP
#include "main.h"
#include "lemlib/api.hpp"
#include <string>

void auton1();
void auton2();
void auton3();
void auton4();
void auton5();
void auton6();
void auton7();
void auton8();
void auton9();
void auton10();

void moveLinear(double inches, int timeout = 2000, float maxspeed = 70, float minspeed = 40);
//void resetOdometry();
void chassisPID(std::string premade = "normal", double lat_kp = chassis.lateralPID.kP, double lat_ki = chassis.lateralPID.kI, double lat_kd = chassis.lateralPID.kD, double lat_slew = chassis.lateralPID.windupRange, double ang_kp = chassis.angularPID.kP, double ang_ki = chassis.angularPID.kI, double ang_kd = chassis.angularPID.kD);

#endif