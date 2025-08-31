#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"

using namespace pahlib;

// A custom struct to hold all four values from the path file
struct RamsetePathPoint {
    double x;
    double y;
    double theta;
    double velocity;
};

/**
 * @brief Parses a VEX Path-Generator asset file into a vector of RamsetePathPoints.
 * Simplified parser for the double pathPoints[][] = {{x,y,theta,vel}, ...} format.
 */
std::vector<RamsetePathPoint> getRamsetePathData(const asset& path) {
    std::vector<RamsetePathPoint> robotPath;

    // Read the entire asset into a single string
    const std::string data(reinterpret_cast<char*>(path.buf), path.size);
    
    // Find the opening brace of the array data
    size_t start_pos = data.find("{{");
    if (start_pos == std::string::npos) {
        return robotPath;
    }
    
    // Find the closing brace of the array data
    size_t end_pos = data.rfind("}}");
    if (end_pos == std::string::npos) {
        return robotPath;
    }
    
    // Extract just the data between the braces
    std::string array_data = data.substr(start_pos + 1, end_pos - start_pos);
    
    size_t pos = 0;
    while (pos < array_data.length()) {
        // Find the next point (enclosed in {})
        size_t point_start = array_data.find('{', pos);
        if (point_start == std::string::npos) break;
        
        size_t point_end = array_data.find('}', point_start);
        if (point_end == std::string::npos) break;
        
        // Extract the point data
        std::string point_str = array_data.substr(point_start + 1, point_end - point_start - 1);
        
        // Parse the four comma-separated values
        std::vector<double> values;
        std::stringstream ss(point_str);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            // Remove whitespace
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (!token.empty()) {
                try {
                    values.push_back(std::stod(token));
                } catch (const std::exception& e) {
                    return std::vector<RamsetePathPoint>(); // Return empty on error
                }
            }
        }
        
        if (values.size() == 4) {
            RamsetePathPoint point;
            point.x = values[0];
            point.y = values[1];
            point.theta = values[2];
            point.velocity = values[3];
            robotPath.push_back(point);
        }
        
        pos = point_end + 1;
    }
    
    return robotPath;
}

void Chassis::ramsete(const asset& path, float beta, float zeta, int timeout, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    
    if (async) {
        pros::Task task([&]() { ramsete(path, beta, zeta, timeout, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    // Parse the path data
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
    int currentTargetIndex = 0; // Track which point we're following
    
    // Velocity scaling factor - adjust based on your units
    // If path velocities are in different units than motor commands, scale here
    const float velocityScale = 1.0f; // Adjust this if needed

    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
        pose = this->getPose(true);
        
        distTraveled += pose.distance(lastPose);
        lastPose = pose;
        
        // Find the target point (look ahead from current target, don't go backwards)
        float minDistance = INFINITY;
        int bestIndex = currentTargetIndex;
        
        // Look for the closest point ahead of our current target
        for (int j = currentTargetIndex; j < pathPoints.size(); j++) {
            Pose targetPose(pathPoints[j].x, pathPoints[j].y);
            float dist = pose.distance(targetPose);
            
            if (dist < minDistance) {
                minDistance = dist;
                bestIndex = j;
            }
            
            // If we're close enough to this point, we can advance our target
            if (dist < 3.0 && j > currentTargetIndex) { // 3 inch lookahead
                currentTargetIndex = j;
            }
        }
        
        // Use the best point we found
        currentTargetIndex = bestIndex;
        
        // Check if we've reached the end
        if (currentTargetIndex >= pathPoints.size() - 1) {
            Pose endPose(pathPoints.back().x, pathPoints.back().y);
            if (pose.distance(endPose) < 2.0) { // 2 inch tolerance
                break;
            }
        }

        const RamsetePathPoint& targetPoint = pathPoints[currentTargetIndex];

        // Desired velocities from the path
        float vd = targetPoint.velocity * velocityScale;
        float wd = 0; // Path doesn't specify angular velocity
        
        // Calculate pose error in global frame
        float error_x_global = targetPoint.x - pose.x;
        float error_y_global = targetPoint.y - pose.y;
        
        // Target heading in radians
        float targetHeading = degToRad(targetPoint.theta);
        
        float e_theta = angleError(targetHeading, pose.theta, true);

        // Transform error to robot's local coordinate frame
        float cos_theta = std::cos(pose.theta);
        float sin_theta = std::sin(pose.theta);
        float e_x = error_x_global * cos_theta + error_y_global * sin_theta;
        float e_y = -error_x_global * sin_theta + error_y_global * cos_theta;

        // Ramsete controller calculations
        float k = 2.0f * zeta * std::sqrt(wd * wd + beta * vd * vd);
        
        // Linear velocity command
        float v_cmd = vd * std::cos(e_theta) + k * e_x;
        
        // Angular velocity command with sinc function
        float sinc_e_theta = (std::abs(e_theta) < 1e-6) ? 1.0f : std::sin(e_theta) / e_theta;
        float w_cmd = wd + k * e_theta + beta * vd * sinc_e_theta * e_y;
        
        // Convert to wheel velocities
        float trackWidth = drivetrain.trackWidth;
        float leftVel = v_cmd - (w_cmd * trackWidth / 2.0f);
        float rightVel = v_cmd + (w_cmd * trackWidth / 2.0f);
        
        // Scale velocities to motor command range [-127, 127]
        float maxVel = std::max(std::abs(leftVel), std::abs(rightVel));
        if (maxVel > 127.0f) {
            leftVel = (leftVel / maxVel) * 127.0f;
            rightVel = (rightVel / maxVel) * 127.0f;
        }
        
        // Apply slew rate limiting
        leftVel = slew(leftVel, prevLeftVel, lateralSettings.slew);
        rightVel = slew(rightVel, prevRightVel, lateralSettings.slew);
        prevLeftVel = leftVel;
        prevRightVel = rightVel;
        
        // Send commands to motors
        drivetrain.leftMotors->move(static_cast<int>(leftVel));
        drivetrain.rightMotors->move(static_cast<int>(rightVel));

        pros::delay(10);
    }
     
    // Stop the robot
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}