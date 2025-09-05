#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "pros/rtos.hpp"

using namespace pahlib;

struct RamsetePathPoint {
    double x;
    double y;
    double theta;
    double velocity;
};

std::vector<RamsetePathPoint> getRamsetePathData(const asset& path) {
    std::vector<RamsetePathPoint> robotPath;
    const std::string data(reinterpret_cast<char*>(path.buf), path.size);

    // Find the start and end of the main path array
    size_t start_pos = data.find("{{");
    if (start_pos == std::string::npos) {
        return robotPath;
    }
    
    size_t end_pos = data.rfind("}}");
    if (end_pos == std::string::npos || end_pos <= start_pos) {
        return robotPath;
    }
    
    // Extract the inner content
    std::string array_data = data.substr(start_pos + 2, end_pos - (start_pos + 2));
    
    // Use a stringstream to parse points separated by '}, {'
    std::stringstream ss(array_data);
    std::string point_str;
    while (std::getline(ss, point_str, '}')) {
        // Remove leading/trailing spaces and the leading '{'
        point_str.erase(0, point_str.find('{') + 1);
        
        std::vector<double> values;
        std::stringstream point_ss(point_str);
        std::string token;
        
        while (std::getline(point_ss, token, ',')) {
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (!token.empty()) {
                try {
                    values.push_back(std::stod(token));
                } catch (const std::exception& e) {
                    // Log an error or handle it more gracefully
                    return std::vector<RamsetePathPoint>();
                }
            }
        }
        
        if (values.size() == 4) {
            RamsetePathPoint point;
            point.x = values[0];
            point.y = values[1];
            point.theta = degToRad(values[2]); // Convert to radians during parsing
            point.velocity = values[3];
            robotPath.push_back(point);
        }
    }
    
    return robotPath;
}

void Chassis::ramsete(const asset& path, float beta, float zeta, int timeout, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    
    if (async) {
        pros::Task task([=, this]() {ramsete(path, beta, zeta, timeout, false);});
        pros::delay(10);
        this->endMotion();
        return;
    }

    std::vector<RamsetePathPoint> pathPoints = getRamsetePathData(path);
    if (pathPoints.empty()) {
        distTraveled = -1;
        this->endMotion();
        return;
    }
     
    Pose pose = this->getPose(true);
    Pose lastPose = pose;
    int compState = pros::competition::get_status();
    distTraveled = 0;
     
    float prevLeftVel = 0;
    float prevRightVel = 0;
    int currentTargetIndex = 0;
    
    const float trackWidth = drivetrain.trackWidth;
    constexpr float maxMotorVoltage = 127.0f;

    // Fixed lookahead distance - crucial for smooth following
    const float lookaheadDistance = 6.0f; // inches

    Timer timer(timeout);

    while (!timer.isDone() && pros::competition::get_status() == compState && this->motionRunning) {
        pose = this->getPose(true);
        
        distTraveled += pose.distance(lastPose);
        lastPose = pose;
        
        // Improved target point selection with lookahead
        float minDistance = INFINITY;
        int bestIndex = currentTargetIndex;
        
        // First find the closest point
        for (int j = currentTargetIndex; j < pathPoints.size(); j++) {
            Pose targetPose(pathPoints[j].x, pathPoints[j].y);
            float dist = pose.distance(targetPose);
            
            if (dist < minDistance) {
                minDistance = dist;
                bestIndex = j;
            } else if (j > bestIndex + 5) {
                // Stop searching if we're moving away for too long
                break;
            }
        }
        
        // Now find the lookahead point
        currentTargetIndex = bestIndex;
        for (int j = bestIndex; j < pathPoints.size(); j++) {
            Pose targetPose(pathPoints[j].x, pathPoints[j].y);
            float dist = pose.distance(targetPose);
            
            if (dist >= lookaheadDistance) {
                currentTargetIndex = j;
                break;
            }
            currentTargetIndex = j; // Use the last point if none meet lookahead
        }
        
        // Ensure we don't go backwards unless very close to a previous point
        if (currentTargetIndex < bestIndex && minDistance > 2.0f) {
            currentTargetIndex = bestIndex;
        }
        
        // Check for end of path
        if (currentTargetIndex >= pathPoints.size() - 1) {
            currentTargetIndex = pathPoints.size() - 1;
            Pose endPose(pathPoints.back().x, pathPoints.back().y);
            if (pose.distance(endPose) < 2.0f) { // Tighter tolerance
                break;
            }
        }

        const RamsetePathPoint& targetPoint = pathPoints[currentTargetIndex];

        float vd = targetPoint.velocity;
        
        float targetHeading = targetPoint.theta; // Already in radians
        float wd = 0.0f;
        
        // Improved angular velocity calculation
        if (currentTargetIndex < pathPoints.size() - 1) {
            float nextHeading = pathPoints[currentTargetIndex + 1].theta;
            wd = angleError(nextHeading, targetHeading, true) * 15.0f; // Increased gain
        }
        
        float error_x_global = targetPoint.x - pose.x;
        float error_y_global = targetPoint.y - pose.y;
        float e_theta = angleError(targetHeading, pose.theta, true);

        // Transform to robot frame
        float cos_theta = std::cos(pose.theta);
        float sin_theta = std::sin(pose.theta);
        float e_x = error_x_global * cos_theta + error_y_global * sin_theta;
        float e_y = -error_x_global * sin_theta + error_y_global * cos_theta;

        // Ramsete controller gains
        float k = 2.0f * zeta * std::sqrt(wd * wd + beta * vd * vd);
        
        float v_cmd = vd * std::cos(e_theta) + k * e_x;
        
        // Improved sinc function calculation
        float sinc_e_theta = (std::abs(e_theta) < 1e-6) ? 1.0f : std::sin(e_theta) / e_theta;
        float w_cmd = wd + k * e_theta + beta * vd * sinc_e_theta * e_y;
        
        // Convert to wheel velocities
        float leftVel = v_cmd - (w_cmd * trackWidth / 2.0f);
        float rightVel = v_cmd + (w_cmd * trackWidth / 2.0f);
        
        // Voltage saturation
        float maxVel = std::max(std::abs(leftVel), std::abs(rightVel));
        if (maxVel > maxMotorVoltage) {
            leftVel = (leftVel / maxVel) * maxMotorVoltage;
            rightVel = (rightVel / maxVel) * maxMotorVoltage;
        }
        
        // Apply slew rate limiter
        leftVel = slew(leftVel, prevLeftVel, lateralSettings.slew);
        rightVel = slew(rightVel, prevRightVel, lateralSettings.slew);
        
        // Improved minimum speed handling
        float distanceToTarget = pose.distance(Pose(targetPoint.x, targetPoint.y));
        const float minSpeed = 10.0f; // Reduced minimum speed
        
        if (distanceToTarget > 3.0f) {
            if (std::abs(leftVel) < minSpeed && std::abs(leftVel) > 1.0f) {
                leftVel = (leftVel >= 0) ? minSpeed : -minSpeed;
            }
            if (std::abs(rightVel) < minSpeed && std::abs(rightVel) > 1.0f) {
                rightVel = (rightVel >= 0) ? minSpeed : -minSpeed;
            }
        }

        prevLeftVel = leftVel;
        prevRightVel = rightVel;
        
        drivetrain.leftMotors->move(static_cast<int>(leftVel));
        drivetrain.rightMotors->move(static_cast<int>(rightVel));

        pros::delay(10);
    }
     
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}