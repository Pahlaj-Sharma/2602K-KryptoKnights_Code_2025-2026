#ifndef SUBSYSTEMS_HPP
#define SUBSYSTEMS_HPP

#include "main.h"
#include "pahlib/api.hpp"

/**
 * @brief Toggles the state of the Power Take-Off (PTO) mechanism.
 *
 * This function changes the global PTO state, sets the digital output
 * to control the physical mechanism, and dynamically adds or removes
 * motors from the drivetrain motor groups.
 *
 * @param state The desired state of the PTO. `true` for intake, `false` for drivetrain.
 */
void toggle_pto(bool state); // Function to toggle the PTO state

void toggle_preroller(bool toggled, int vel = 90); // Function to toggle the preroller

void toggle_score(bool toggled, int vel = 90, int topVel = 90); // Function to toggle the scoring motor

#endif