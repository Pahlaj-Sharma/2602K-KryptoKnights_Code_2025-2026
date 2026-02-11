#include "pahlib/chassis/chassis.hpp"
#include "main.h"
#include "pahlib/util.hpp"
#include "robot_config.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace {
    // Constants
    constexpr float MM_TO_INCH = 0.03937;
    constexpr float ANGLE_TOLERANCE = 0.174533; // ~10 degrees in radians
    constexpr float DISTANCE_TOLERANCE = 5.0; // inches
    constexpr float MAX_VALID_DISTANCE = 25.0; // inches
    
    // Field walls (in inches, centered coordinate system)
    constexpr float WALL_0_X = 70.2;   // Right wall (+X)
    constexpr float WALL_1_Y = 70.2;   // Front wall (+Y)
    constexpr float WALL_2_X = -70.2;  // Left wall (-X)
    constexpr float WALL_3_Y = -70.2;  // Back wall (-Y)
    
    enum class Axis { NONE, X, Y };
    
    struct SensorConfig {
        pros::Distance& sensor;
        float offsetX;  // Sensor offset from robot center in X (robot frame)
        float offsetY;  // Sensor offset from robot center in Y (robot frame)
        float offsetTheta; // Sensor angle offset in radians (robot frame)
    };
    
    struct DistanceResult {
        Axis axis;
        float axisPosition;
        float confidence;
        
        DistanceResult() : axis(Axis::NONE), axisPosition(0), confidence(0) {}
        DistanceResult(Axis a, float pos, float conf) 
            : axis(a), axisPosition(pos), confidence(conf) {}
    };
    
    // Get sensor reading and calculate field position
    DistanceResult getReading(const pahlib::Pose& robotPose, const SensorConfig& sensor, bool force) {
        DistanceResult result;
        
        // Get raw distance reading
        float realReading = sensor.sensor.get() * MM_TO_INCH;
        
        // Validate distance is reasonable
        if (realReading <= 0 || realReading > MAX_VALID_DISTANCE) {
            return result; // Invalid reading
        }
        
        // Calculate sensor angle in field frame (standard position: 90° = forward)
        float sensorAngle = robotPose.theta + sensor.offsetTheta;
        
        // Transform sensor position from robot frame to field frame
        float sensorOffsetX = sensor.offsetX * std::cos(robotPose.theta) - 
                              sensor.offsetY * std::sin(robotPose.theta);
        float sensorOffsetY = sensor.offsetX * std::sin(robotPose.theta) + 
                              sensor.offsetY * std::cos(robotPose.theta);
        
        float sensorX = robotPose.x + sensorOffsetX;
        float sensorY = robotPose.y + sensorOffsetY;
        
        // Determine which wall this sensor should be reading
        // Check angle to each wall and find the closest match
        float predictedReading = 0;
        float angleError = 0;
        int wall = -1;
        
        // Wall 0: Right wall (+X direction, sensor pointing at 0°)
        angleError = std::remainder(sensorAngle, M_TWOPI);
        if (std::abs(angleError) < ANGLE_TOLERANCE) {
            predictedReading = (WALL_0_X - sensorX) / std::cos(angleError);
            wall = 0;
        }
        // Wall 1: Front wall (+Y direction, sensor pointing at 90°/π/2)
        else if (angleError = std::remainder(M_PI_2 - sensorAngle, M_TWOPI); 
                 std::abs(angleError) < ANGLE_TOLERANCE) {
            predictedReading = (WALL_1_Y - sensorY) / std::cos(angleError);
            wall = 1;
        }
        // Wall 2: Left wall (-X direction, sensor pointing at 180°/π)
        else if (angleError = std::remainder(M_PI - sensorAngle, M_TWOPI); 
                 std::abs(angleError) < ANGLE_TOLERANCE) {
            predictedReading = (sensorX - WALL_2_X) / std::cos(angleError);
            wall = 2;
        }
        // Wall 3: Back wall (-Y direction, sensor pointing at 270°/3π/2)
        else if (angleError = std::remainder(M_3PI_2 - sensorAngle, M_TWOPI); 
                 std::abs(angleError) < ANGLE_TOLERANCE) {
            predictedReading = (sensorY - WALL_3_Y) / std::cos(angleError);
            wall = 3;
        }
        else {
            return result; // Not aligned with any wall
        }
        
        // Check if predicted and actual readings match (unless forced)
        if (!force && std::abs(realReading - predictedReading) > DISTANCE_TOLERANCE) {
            return result; // Reading doesn't match expected distance to wall
        }
        
        // Calculate position based on wall and reading
        if (wall == 0) {
            result.axis = Axis::X;
            result.axisPosition = (WALL_0_X - realReading * std::cos(angleError)) - sensorOffsetX;
        } 
        else if (wall == 1) {
            result.axis = Axis::Y;
            result.axisPosition = (WALL_1_Y - realReading * std::cos(angleError)) - sensorOffsetY;
        } 
        else if (wall == 2) {
            result.axis = Axis::X;
            result.axisPosition = (WALL_2_X + realReading * std::cos(angleError)) - sensorOffsetX;
        } 
        else if (wall == 3) {
            result.axis = Axis::Y;
            result.axisPosition = (WALL_3_Y + realReading * std::cos(angleError)) - sensorOffsetY;
        }
        
        // Calculate confidence (closer readings and smaller angle errors = higher confidence)
        result.confidence = (MAX_VALID_DISTANCE - realReading) / MAX_VALID_DISTANCE * 
                           (ANGLE_TOLERANCE - std::abs(angleError)) / ANGLE_TOLERANCE;
        
        return result;
    }
}

void pahlib::Chassis::resetOdometry(float threshold, bool force) {
    // Get current robot pose (in radians, standard position for calculations)
    const pahlib::Pose robotPose = this->getPose(true, true);
    
    // Also get theta in robot position format for setPose later
    const pahlib::Pose robotPoseRobotFrame = this->getPose();
    
    // Configure all sensors with their physical offsets
    // offsetTheta is relative to robot's forward direction (0° = forward, 90° = left, etc.)
    std::vector<SensorConfig> sensors = {
        {frontDistance, DS_FRONT_CENTER, DS_FRONT_CENTER_2, 0},              // Front sensor (0° relative to robot)
        {backDistance,  DS_BACK_CENTER,  DS_BACK_CENTER_2,  M_PI},           // Back sensor (180° relative to robot)
        {leftDistance,  DS_LEFT_CENTER,  DS_LEFT_CENTER_2,  -M_PI / 2},      // Left sensor (-90° relative to robot)
        {rightDistance, DS_RIGHT_CENTER, DS_RIGHT_CENTER_2, M_PI / 2}        // Right sensor (90° relative to robot)
    };
    
    // Collect valid readings from all sensors
    std::vector<DistanceResult> resultsX, resultsY;
    
    for (const auto& sensor : sensors) {
        DistanceResult result = getReading(robotPose, sensor, force);
        
        if (result.axis == Axis::X) {
            resultsX.push_back(result);
        } 
        else if (result.axis == Axis::Y) {
            resultsY.push_back(result);
        }
    }
    
    // Don't reset if we have no valid readings
    if (resultsX.empty() && resultsY.empty()) {
        return;
    }
    
    // Sort results by confidence (highest first)
    std::sort(resultsX.begin(), resultsX.end(), 
              [](const DistanceResult& a, const DistanceResult& b) {
                  return a.confidence > b.confidence;
              });
    
    std::sort(resultsY.begin(), resultsY.end(), 
              [](const DistanceResult& a, const DistanceResult& b) {
                  return a.confidence > b.confidence;
              });
    
    // Use the highest confidence reading for each axis
    float newX = robotPose.x;
    float newY = robotPose.y;
    bool updateX = false, updateY = false;
    
    if (!resultsX.empty()) {
        float calcX = resultsX[0].axisPosition;
        if (std::abs(calcX - robotPose.x) < threshold) {
            newX = calcX;
            updateX = true;
        }
    }
    
    if (!resultsY.empty()) {
        float calcY = resultsY[0].axisPosition;
        if (std::abs(calcY - robotPose.y) < threshold) {
            newY = calcY;
            updateY = true;
        }
    }
    
    // Update pose if at least one axis is valid
    // Use robot position format theta for setPose
    if (updateX || updateY) {
        this->setPose(newX, newY, robotPoseRobotFrame.theta);
    }
}