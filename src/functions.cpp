#include "main.h"
#include "lemlib/api.hpp"
#include "robot_config.hpp"
#include "functions.hpp"
#include <cmath>
#include <tuple>
#include <vector>

// --- PID Tuning Components ---
// These components are for live PID tuning and should be removed
// once tuning is complete.
pros::Rotation rot_kp(1);
pros::Rotation rot_ki(2);
pros::Rotation rot_kd(3);
pros::adi::DigitalIn limit_switch(4);

void moveLinear(double inches, int timeout, float lead, float maxspeed, float minspeed) {
    // Get the robot's current position and orientation
    const lemlib::Pose currentPose = chassis.getPose(true);

    // Calculate the target position based on the current pose and distance
    const double targetX = currentPose.x + (inches * std::cos(currentPose.theta));
    const double targetY = currentPose.y + (inches * std::sin(currentPose.theta));

    // Set the chassis to move to the calculated target position
    chassis.moveToPose(targetX, targetY, currentPose.theta, timeout, {
        .lead = lead,
        .maxSpeed = maxspeed,
        .minSpeed = minspeed
    });
}

void chassisPID(PIDPreset premade) {
    PIDConstants lateral_pid;
    PIDConstants angular_pid;

    switch (premade) {
        case PIDPreset::normal:
            lateral_pid = LATERAL_PID;
            angular_pid = ANGULAR_PID;
            break;
        case PIDPreset::fast:
            lateral_pid = F_LATERAL_PID;
            angular_pid = F_ANGULAR_PID;
            break;
        case PIDPreset::precise:
            lateral_pid = P_LATERAL_PID;
            angular_pid = P_ANGULAR_PID;
            break;
        default:
            // Fallback to normal PID if an invalid preset is provided
            lateral_pid = LATERAL_PID;
            angular_pid = ANGULAR_PID;
            break;
    }

    chassis.lateralPID = {lateral_pid.kP, lateral_pid.kI, lateral_pid.kD};
    chassis.angularPID = {angular_pid.kP, angular_pid.kI, angular_pid.kD};
}

void chassisPID(float lat_kp, float lat_ki, float lat_kd, float ang_kp, float ang_ki, float ang_kd) {
    chassis.lateralPID = {lat_kp, lat_ki, lat_kd};
    chassis.angularPID = {ang_kp, ang_ki, ang_kd};
}

struct SensorData {
    double distance;
    double offset;
    char axis; // 'X' or 'Y'

    // Comparison operator to sort by distance
    bool operator<(const SensorData& other) const {
        return this->distance < other.distance;
    }
};

void resetOdometry(double threshold) {
    constexpr double MM_IN = 0.03937;
    constexpr double FIELD_SIZE = 70.0; // Field size in inches

    // Get the initial pose once for consistency
    const lemlib::Pose initial_pose = chassis.getPose(true);

    // Populate a vector of SensorData structs for clarity
    std::vector<SensorData> sensor_readings = {
        {frontDistance.get() * MM_IN, DS_FRONT_CENTER, 'Y'},
        {backDistance.get() * MM_IN, DS_BACK_CENTER, 'Y'},
        {leftDistance.get() * MM_IN, DS_LEFT_CENTER, 'X'},
        {rightDistance.get() * MM_IN, DS_RIGHT_CENTER, 'X'}
    };

    // Sort the sensor readings based on distance from closest to farthest
    std::sort(sensor_readings.begin(), sensor_readings.end());

    // Abort if the two closest sensors are on the same axis or if the closest sensor is too far
    if (sensor_readings[0].axis == sensor_readings[1].axis || sensor_readings[0].distance > 10.0) {
        return;
    }

    // Get the two closest sensors
    const SensorData& sensor1 = sensor_readings[0];
    const SensorData& sensor2 = sensor_readings[1];

    // Calculate the total distances from the wall
    const double distance1 = sensor1.distance + sensor1.offset;
    const double distance2 = sensor2.distance + sensor2.offset;

    double calculated_x, calculated_y;

    // Use an intuitive variable name for the condition
    const bool standard_angle_axis = (0 <= initial_pose.theta && initial_pose.theta < M_PI_4) ||
                                     (M_7PI_4 < initial_pose.theta && initial_pose.theta <= M_2_PI) ||
                                     (M_3PI_4 < initial_pose.theta && initial_pose.theta < M_5PI_4);

    if (standard_angle_axis) {
        if (sensor1.axis == 'X') {
            calculated_x = distance1;
            calculated_y = distance2;
        } else {
            calculated_x = distance2;
            calculated_y = distance1;
        }
    } else { // Other angles
        if (sensor1.axis == 'X') {
            calculated_x = distance2;
            calculated_y = distance1;
        } else {
            calculated_x = distance1;
            calculated_y = distance2;
        }
    }

    // Adjust the calculated position based on the robot's current quadrant
    if (initial_pose.x > 0) {
        calculated_x = FIELD_SIZE - calculated_x;
    } else if (initial_pose.x < 0) {
        calculated_x -= FIELD_SIZE;
    }

    if (initial_pose.y > 0) {
        calculated_y = FIELD_SIZE - calculated_y;
    } else if (initial_pose.y < 0) {
        calculated_y -= FIELD_SIZE;
    }

    // Check if the calculated pose is within the allowed threshold
    const double x_threshold_diff = std::abs(calculated_x - initial_pose.x);
    const double y_threshold_diff = std::abs(calculated_y - initial_pose.y);
    const lemlib::Pose current_pose = chassis.getPose();

    // Set the pose if the difference is within the threshold
    if ((x_threshold_diff < threshold) && (y_threshold_diff < threshold)) {
        chassis.setPose(calculated_x, calculated_y, current_pose.theta);
    } else if (x_threshold_diff < threshold) {
        chassis.setPose(calculated_x, current_pose.y, current_pose.theta);
    } else if (y_threshold_diff < threshold) {
        chassis.setPose(current_pose.x, calculated_y, current_pose.theta);
    }
}

void tunePID() {
    // Store initial PID and rotation sensor values
    const double initial_kp = chassis.lateralPID.kP;
    const double initial_ki = chassis.lateralPID.kI;
    const double initial_kd = chassis.lateralPID.kD;
    const int initial_rot_kp_pos = rot_kp.get_position();
    const int initial_rot_ki_pos = rot_ki.get_position();
    const int initial_rot_kd_pos = rot_kd.get_position();

    // Scaling factors for PID tuning
    constexpr float KP_SCALE_FACTOR = 0.1f;
    constexpr float KI_SCALE_FACTOR = 0.01f;
    constexpr float KD_SCALE_FACTOR = 0.1f;

    while (true) {
        // Calculate the change in PID values based on rotation sensor position
        const double delta_kp = (rot_kp.get_position() - initial_rot_kp_pos) * KP_SCALE_FACTOR;
        const double delta_ki = (rot_ki.get_position() - initial_rot_ki_pos) * KI_SCALE_FACTOR;
        const double delta_kd = (rot_kd.get_position() - initial_rot_kd_pos) * KD_SCALE_FACTOR;

        // Apply the changes to the lateral PID constants
        // This can be swapped for angularPID to tune turning
        chassis.lateralPID.kP = initial_kp + delta_kp;
        chassis.lateralPID.kI = initial_ki + delta_ki;
        chassis.lateralPID.kD = initial_kd + delta_kd;

        // Print the current PID values to the controller screen
        controller.print(0, 0, "kP: %f", chassis.lateralPID.kP);
        controller.print(1, 0, "kI: %f", chassis.lateralPID.kI);
        controller.print(2, 0, "kD: %f", chassis.lateralPID.kD);

        // Check if the limit switch is pressed to run a test movement
        if (limit_switch.get_new_press()) {
            controller.rumble("-"); // Short rumble to indicate start of test
            chassis.calibrate();
            chassis.setPose(0, 0, 0);
            pros::delay(100);
            chassis.moveToPoint(0, 24, 10000); // Test movement (change as needed)
            chassis.waitUntilDone();
            controller.rumble("."); // Long rumble to indicate end of test
            controller.clear();
        }

        // Small delay to prevent a task overflow
        pros::delay(100);
    }
}