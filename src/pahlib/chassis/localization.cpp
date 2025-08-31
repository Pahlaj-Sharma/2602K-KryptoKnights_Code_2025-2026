#include "main.h"
#include "robot_config.hpp"

struct SensorData {
    float distance, offset;
    char axis; // 'X' or 'Y'

    // Comparison operator to sort by distance
    bool operator<(const SensorData& other) const {
        return this->distance < other.distance;
    }
};

void pahlib::Chassis::resetOdometry(float threshold) {
    constexpr float MM_TO_INCH = 0.03937f;
    constexpr float FIELD_SIZE_IN = 70.0f;

    // Get the robot's current pose
    const pahlib::Pose pose = this->getPose(true);

    // Gather sensor readings (distance in inches, offset, axis)
    std::vector<SensorData> sensors = {
        {frontDistance.get() * MM_TO_INCH, DS_FRONT_CENTER, 'Y'},
        {backDistance.get()  * MM_TO_INCH, DS_BACK_CENTER,  'Y'},
        {leftDistance.get()  * MM_TO_INCH, DS_LEFT_CENTER,  'X'},
        {rightDistance.get() * MM_TO_INCH, DS_RIGHT_CENTER, 'X'}
    };

    // Sort sensors by distance (closest first)
    std::sort(sensors.begin(), sensors.end());

    // Abort if the two closest sensors are on the same axis, too far, or robot is moving
    if (sensors[0].axis == sensors[1].axis ||
        sensors[0].distance > 8.0f ||
        this->isInMotion()) {
        return;
    }

    // Assign the two closest sensors
    const SensorData& s1 = sensors[0];
    const SensorData& s2 = sensors[1];

    // Calculate orientation scaling factor
    const float orientation_scale = std::max(std::abs(std::cos(pose.theta)),
                                             std::abs(std::sin(pose.theta)));

    // Compute wall distances for both sensors
    const float dist1 = (s1.distance + s1.offset) * orientation_scale;
    const float dist2 = (s2.distance + s2.offset) * orientation_scale;

    float calc_x = 0, calc_y = 0;

    // Determine if the robot is facing a standard axis direction
    const bool standard_axis = (
        (0.0f <= pose.theta && pose.theta < M_PI_4) ||
        (M_7PI_4 < pose.theta && pose.theta <= M_TWOPI) ||
        (M_3PI_4 < pose.theta && pose.theta < M_5PI_4)
    );

    // Assign calculated x and y based on sensor axes and orientation
    if (s1.axis == 'X') {
        calc_x = standard_axis ? dist1 : dist2;
        calc_y = standard_axis ? dist2 : dist1;
    } else {
        calc_x = standard_axis ? dist2 : dist1;
        calc_y = standard_axis ? dist1 : dist2;
    }

    // Adjust calculated position based on robot's current quadrant
    if (pose.x > 0.0f)
        calc_x = FIELD_SIZE_IN - calc_x;
    else if (pose.x < 0.0f)
        calc_x -= FIELD_SIZE_IN;
    else
        calc_x = 0.0f;

    if (pose.y > 0.0f)
        calc_y = FIELD_SIZE_IN - calc_y;
    else if (pose.y < 0.0f)
        calc_y -= FIELD_SIZE_IN;
    else
        calc_y = 0.0f;

    // Check if the new pose is within the allowed threshold
    const float x_diff = std::abs(calc_x - pose.x);
    const float y_diff = std::abs(calc_y - pose.y);

    // Set the new pose if within threshold
    if (x_diff < threshold && y_diff < threshold) 
        this->setPose(calc_x, calc_y, pose.theta);
    else if (x_diff < threshold) 
        this->setPose(calc_x, pose.y, pose.theta);
    else if (y_diff < threshold) 
        this->setPose(pose.x, calc_y, pose.theta);
    else return;
}
