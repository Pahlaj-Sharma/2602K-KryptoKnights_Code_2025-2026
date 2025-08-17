#include "main.h"
#include "lemlib/api.hpp"
#include "robot_config.hpp"
#include "functions.hpp"

// --- PID Tuning Components ---
// These components are for live PID tuning and should be removed once tuning is complete.
pros::Rotation rot_kp(1);
pros::Rotation rot_ki(2);
pros::Rotation rot_kd(3);

void moveLinear(float inches, int timeout, float lead, float maxspeed, float minspeed) {
    // Get the robot's current position and orientation
    const lemlib::Pose currentPose = chassis.getPose(true);

    // Calculate the target position based on the current pose and distance
    const float targetX = currentPose.x + (inches * std::cos(currentPose.theta));
    const float targetY = currentPose.y + (inches * std::sin(currentPose.theta));

    // Set the chassis to move to the calculated target position
    chassis.moveToPose(targetX, targetY, lemlib::radToDeg(currentPose.theta), timeout, {
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
    float distance, offset;
    char axis; // 'X' or 'Y'

    // Comparison operator to sort by distance
    bool operator<(const SensorData& other) const {
        return this->distance < other.distance;
    }
};

void resetOdometry(float threshold) {
    constexpr float MM_IN = 0.03937, FIELD_SIZE_IN = 70.0;

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
    if (sensor_readings[0].axis == sensor_readings[1].axis || sensor_readings[0].distance > 8.0 || chassis.isInMotion()) return;

    // Get the two closest sensors
    const SensorData& sensor1 = sensor_readings[0];
    const SensorData& sensor2 = sensor_readings[1];

    // Calculate the total distances from the wall
    const float max_theta = std::max(std::abs(std::cos(initial_pose.theta)),
                                     std::abs(std::sin(initial_pose.theta)));
    
    const float distance1 = sensor1.distance * max_theta + sensor1.offset * max_theta;
    const float distance2 = sensor2.distance * max_theta + sensor2.offset * max_theta;

    float calculated_x, calculated_y;

    const bool standard_angle_axis = (0.0 <= initial_pose.theta && initial_pose.theta < M_PI_4) ||
                                     (M_7PI_4 < initial_pose.theta && initial_pose.theta <= M_TWOPI) ||
                                     (M_3PI_4 < initial_pose.theta && initial_pose.theta < M_5PI_4);

    if (sensor1.axis == 'X') {
        calculated_x = (standard_angle_axis) ? distance1 : distance2;
        calculated_y = (standard_angle_axis) ? distance2 : distance1;
    } else {
        calculated_x = (standard_angle_axis) ? distance2 : distance1;
        calculated_y = (standard_angle_axis) ? distance1 : distance2;
    }
    
    // Adjust the calculated position based on the robot's current quadrant
    if (initial_pose.x > 0) {
        calculated_x = FIELD_SIZE_IN - calculated_x;
    } else if (initial_pose.x < 0) {
        calculated_x -= FIELD_SIZE_IN;
    } else calculated_x = 0;

    if (initial_pose.y > 0) {
        calculated_y = FIELD_SIZE_IN - calculated_y;
    } else if (initial_pose.y < 0) {
        calculated_y -= FIELD_SIZE_IN;
    } else calculated_y = 0;

    // Check if the calculated pose is within the allowed threshold
    const float x_threshold_diff = std::abs(calculated_x - initial_pose.x);
    const float y_threshold_diff = std::abs(calculated_y - initial_pose.y);

    // Set the pose if the difference is within the threshold
    if ((x_threshold_diff < threshold) && (y_threshold_diff < threshold)) {
        chassis.setPose(calculated_x, calculated_y, chassis.getPose().theta);
    } else if (x_threshold_diff < threshold) {
        chassis.setPose(calculated_x, chassis.getPose().y, chassis.getPose().theta);
    } else if (y_threshold_diff < threshold) {
        chassis.setPose(chassis.getPose().x, calculated_y, chassis.getPose().theta);
    } else return;
}

void tunePID() {
    // Store initial PID and rotation sensor values
    const int initial_rot_kp_pos = rot_kp.get_position();
    const int initial_rot_ki_pos = rot_ki.get_position();
    const int initial_rot_kd_pos = rot_kd.get_position();

    // Scaling factors for PID tuning
    constexpr float KP_SCALE_FACTOR = 0.1f, KI_SCALE_FACTOR = 0.01f, KD_SCALE_FACTOR = 0.1f;

    while (true) {
        // Calculate the change in PID values based on rotation sensor position
        const float delta_kp = (rot_kp.get_position() - initial_rot_kp_pos) * KP_SCALE_FACTOR;
        const float delta_ki = (rot_ki.get_position() - initial_rot_ki_pos) * KI_SCALE_FACTOR;
        const float delta_kd = (rot_kd.get_position() - initial_rot_kd_pos) * KD_SCALE_FACTOR;

        // Apply the changes to the lateral PID constants
        chassis.lateralPID.kP += delta_kp;
        chassis.lateralPID.kI += delta_ki;
        chassis.lateralPID.kD += delta_kd;

        // Print the current PID values to the controller screen
        controller.print(0, 0, "kP: %f", chassis.lateralPID.kP);
        controller.print(1, 0, "kI: %f", chassis.lateralPID.kI);
        controller.print(2, 0, "kD: %f", chassis.lateralPID.kD);

        // Check if the limit switch is pressed to run a test movement
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            controller.rumble("-");
            chassis.calibrate();
            chassis.setPose(0, 0, 0);
            pros::delay(100);
            chassis.moveToPoint(0, 24, 10000); // Test movement (change as needed)
            chassis.waitUntilDone();
            controller.rumble(".");
            controller.clear();
        }
        pros::delay(50);
    }
}
