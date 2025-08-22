/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#include "pahlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#include "pahlib/api.hpp"
extern pahlib::Chassis chassis;
extern pros::Controller controller;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup left_motors;
extern pros::Motor left_pto;
extern pros::Motor right_pto;
extern pros::Distance rightDistance;
extern pros::Distance leftDistance;
extern pros::Distance frontDistance;
extern pros::Distance backDistance;
extern pros::adi::DigitalOut pto;
extern int selectedAuton;
extern bool ptoState; // PTO state, false = drivetrain, true = intake

class ScalarIMU : public pros::IMU {
public:
    /**
     * @brief Constructor for the CustomIMU class.
     * @param port The V5 port the IMU is connected to.
     * @param scalar A scalar value to apply to the rotation reading.
     */
    ScalarIMU(int port, float scalar)
    : pros::IMU(port),
    m_port(port),
    m_scalar(scalar) {}

    /**
     * @brief Gets the scaled rotation of the IMU.
     * @return The rotation value multiplied by the scalar.
     */
    virtual double get_rotation() const {
        return pros::c::imu_get_rotation(m_port) * m_scalar;
    }

private:
    const int m_port;
    const double m_scalar;
};

// Declare the global instance of your CustomIMU
// This tells other files that 'inertial' exists and will be defined elsewhere.
extern ScalarIMU inertial;

#endif

#endif  // _PROS_MAIN_H_
