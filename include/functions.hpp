#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include "main.h"
#include "lemlib/api.hpp"
#include <string>

/**
 * @brief Enum class for selecting a predefined PID preset.
 *
 * This allows for easy switching between different sets of PID constants
 * for various autonomous tasks (e.g., normal, fast, or precise).
 */
enum class PIDPreset {
    normal,  // Standard PID constants for general movement.
    fast,    // PID constants optimized for high-speed, less precise movements.
    precise  // PID constants optimized for slow, highly precise movements.
};

/**
 * @brief Moves the robot linearly for a specified distance.
 *
 * @param inches The distance to move in inches.
 * @param timeout The maximum time (in milliseconds) to wait for the movement. Defaults to 2000.
 * @param lead The lead parameter for the movement. Defaults to 0.2.
 * @param maxspeed The maximum speed of the chassis. Defaults to 70.
 * @param minspeed The minimum speed of the chassis. Defaults to 40.
 */
void moveLinear(float inches, int timeout = 2000, float lead = 0.1, float maxspeed = 70, float minspeed = 40);

/**
 * @brief Sets the chassis PID constants using a predefined preset.
 *
 * @param premade The predefined PID preset to use (e.g., PIDPreset::normal).
 */
void chassisPID(PIDPreset premade);

/**
 * @brief Sets the chassis PID constants manually.
 *
 * @param lat_kp The kP constant for the lateral controller.
 * @param lat_ki The kI constant for the lateral controller.
 * @param lat_kd The kD constant for the lateral controller.
 * @param ang_kp The kP constant for the angular controller.
 * @param ang_ki The kI constant for the angular controller.
 * @param ang_kd The kD constant for the angular controller.
 */
void chassisPID(float lat_kp, float lat_ki, float lat_kd, float ang_kp, float ang_ki, float ang_kd);

/**
 * @brief Resets the odometry pose based on sensor readings from distance sensors.
 *
 * @param threshold The maximum allowed difference (in inches) between the calculated and current pose. Defaults to 5.0.
 *
 * @param async Whether to run the reset operation asynchronously. Defaults to false.
 */
void resetOdometry(float threshold = 5.0);

#endif