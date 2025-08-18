#include <cmath>
#include <vector>
#include <string>
#include "pros/misc.hpp"
#include "pahlib/logger/logger.hpp"
#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"

void pahlib::Chassis::ramsete(const asset& path, float beta, float zeta, int timeout, bool forwards, bool async) {

    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([&]() { ramsete(path, beta, zeta, timeout, forwards, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    std::vector<pahlib::Pose> pathPoints = getData(path);
    if (pathPoints.size() == 0) {
        pahlib::infoSink()->error("No points in path! Do you have the right format? Skipping motion");
        distTraveled = -1;
        this->endMotion();
        return;
    }
    
    pahlib::Pose pose = this->getPose(true);
    pahlib::Pose lastPose = pose;
    int compState = pros::competition::get_status();
    distTraveled = 0;
    
    float prevLeftVel = 0; // Initialize previous velocities
    float prevRightVel = 0;

    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
        pose = this->getPose(true);
        if (!forwards) pose.theta -= M_PI;

        distTraveled += pose.distance(lastPose);
        lastPose = pose;
        
        // Find the closest point on the path to the robot
        int closestPointIndex;
        float closestDist = infinity();
        for (int j = 0; j < pathPoints.size(); j++) {
            const float dist = pose.distance(pathPoints.at(j));
            if (dist < closestDist) {
                closestDist = dist;
                closestPointIndex = j;
            }
        }
        
        // Check if the robot is at the end of the path
        if (closestPointIndex >= pathPoints.size() - 1) {
            break;
        }

        // Get the current target point and the next point on the path
        pahlib::Pose targetPose = pathPoints.at(closestPointIndex);
        pahlib::Pose nextPose = pathPoints.at(closestPointIndex + 1);

        // Interpolate to find a more precise target point along the segment
        float distToTarget = pose.distance(targetPose);
        float segmentLength = targetPose.distance(nextPose);
        float t = distToTarget / segmentLength;
        pahlib::Pose interpolatedTarget = targetPose.lerp(nextPose, t);
        
        // Use the velocity from the target point on the path
        float targetLinearVel = targetPose.theta;

        // Calculate errors in the global coordinate system
        float errorX = interpolatedTarget.x - pose.x;
        float errorY = interpolatedTarget.y - pose.y;

        // Calculate heading error, properly wrapping the angle
        float headingError = pahlib::angleError(interpolatedTarget.theta, pose.theta, true);

        // Convert errors to the robot's local frame
        float localErrorX = errorX * cos(pose.theta) + errorY * sin(pose.theta);
        float localErrorY = -errorX * sin(pose.theta) + errorY * cos(pose.theta);

        // Ramsete controller calculations
        float k = 2.0 * zeta * sqrt(pow(targetLinearVel, 2) + beta * pow(targetLinearVel, 2) * pow(localErrorY, 2));

        float V = targetLinearVel * cos(headingError) + k * localErrorX;

        float W = 0; // Angular velocity
        if (std::abs(headingError) < 1e-6) {
            W = targetLinearVel * beta * k * localErrorY + k * headingError;
        } else {
            W = targetLinearVel * (sin(headingError) / headingError) * beta * k * localErrorY + k * headingError;
        }

        // Convert velocities to motor commands (127 max)
        float targetLeftVel = V - W * drivetrain.trackWidth / 2.0;
        float targetRightVel = V + W * drivetrain.trackWidth / 2.0;

        // Ratio the speeds to respect the max speed
        float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }

        // Slew rate control to prevent sudden changes in velocity
        targetLeftVel = slew(targetLeftVel, prevLeftVel, lateralSettings.slew);
        targetRightVel = slew(targetRightVel, prevRightVel, lateralSettings.slew);
        prevLeftVel = targetLeftVel;
        prevRightVel = targetRightVel;
        
        // Move the drivetrain based on direction
        if (forwards) {
            drivetrain.leftMotors->move(targetLeftVel);
            drivetrain.rightMotors->move(targetRightVel);
        } else {
            drivetrain.leftMotors->move(-targetRightVel);
            drivetrain.rightMotors->move(-targetLeftVel);
        }

        pros::delay(10);
    }
    
    // Stop the robot
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}

