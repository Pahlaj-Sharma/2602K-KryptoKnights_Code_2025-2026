#include <cmath>
#include <vector>
#include <string>
#include "pros/misc.hpp"
#include "pahlib/logger/logger.hpp"
#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"

// Does not work

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
     
    float prevLeftVel = 0;
    float prevRightVel = 0;

    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
        pose = this->getPose(true);
        // If driving backwards, flip the robot's perceived heading by 180 degrees
        if (!forwards) pose.theta -= M_PI;

        distTraveled += pose.distance(lastPose);
        lastPose = pose;
         
        // Find the closest point on the path to the robot
        int closestPointIndex = 0;
        float closestDist = infinity();
        for (int j = 0; j < pathPoints.size(); j++) {
            const float dist = pose.distance(pathPoints.at(j));
            if (dist < closestDist) {
                closestDist = dist;
                closestPointIndex = j;
            }
        }
         
        // Check if the robot has reached the end of the path
        if (closestPointIndex >= pathPoints.size() - 1 && pose.distance(pathPoints.back()) < 0.1) {
            break;
        }

        pahlib::Pose desiredPose = pathPoints.at(closestPointIndex);

        float vd = desiredPose.theta;
        float wd = 0; // Desired angular velocity (rad/s)

        // Calculate desired angular velocity from the path's curvature
        if (closestPointIndex < pathPoints.size() - 1) {
            pahlib::Pose nextPose = pathPoints.at(closestPointIndex + 1);
            float segmentLength = desiredPose.distance(nextPose);
            if (std::abs(segmentLength) > 1e-6) {
                float headingChange = pahlib::angleError(nextPose.theta, desiredPose.theta, true);
                float curvature = headingChange / segmentLength;
                wd = vd * curvature;
            }
        }
        
        // Compute error in the global coordinate frame
        float error_x_global = desiredPose.x - pose.x;
        float error_y_global = desiredPose.y - pose.y;
        float e_theta = pahlib::angleError(desiredPose.theta, pose.theta, true);

        // Transform global error to the robot's local frame
        float cos_theta_actual = std::cos(pose.theta);
        float sin_theta_actual = std::sin(pose.theta);
        float e_x = error_x_global * cos_theta_actual + error_y_global * sin_theta_actual;
        float e_y = -error_x_global * sin_theta_actual + error_y_global * cos_theta_actual;

        // Compute the controller gain 'k' using the formula from the documentation
        // Formula: k = 2 * ζ * sqrt(ωd^2 + b * vd^2)
        float k = 2.0 * zeta * std::sqrt(std::pow(wd, 2) + beta * std::pow(vd, 2));

        // 5. Compute the required linear (v) and angular (ω) velocities
        // Formula for linear velocity: v = vd * cos(eθ) + k * ex
        float v_out = vd * std::cos(e_theta) + k * e_x;

        // Formula for angular velocity: ω = ωd + k*eθ + (b*vd*sinc(eθ)*ey)
        float sinc_val = 1.0f;
        if (std::abs(e_theta) > 1e-6) { // Use sinc(x) = sin(x)/x to avoid division by zero
            sinc_val = std::sin(e_theta) / e_theta;
        }
        float w_out = wd + k * e_theta + (beta * vd * sinc_val * e_y);
    
        // 6. Convert chassis velocities to left and right wheel velocities
        // NOTE: This assumes vd, wd were in units compatible with motor commands.
        float trackWidth = drivetrain.trackWidth;
        float targetLeftVel = v_out - (w_out * trackWidth / 2.0);
        float targetRightVel = v_out + (w_out * trackWidth / 2.0);

        // 7. Normalize wheel velocities to fit within the motor's command range [-127, 127]
        float max_abs_vel = std::max(std::abs(targetLeftVel), std::abs(targetRightVel));
        if (max_abs_vel > 127.0) {
            targetLeftVel = (targetLeftVel / max_abs_vel) * 127.0;
            targetRightVel = (targetRightVel / max_abs_vel) * 127.0;
        }

        // 8. Apply slew rate control to smooth out acceleration
        targetLeftVel = slew(targetLeftVel, prevLeftVel, lateralSettings.slew);
        targetRightVel = slew(targetRightVel, prevRightVel, lateralSettings.slew);
        prevLeftVel = targetLeftVel;
        prevRightVel = targetRightVel;
         
        // 9. Send final commands to the motors
        if (forwards) {
            drivetrain.leftMotors->move(targetLeftVel);
            drivetrain.rightMotors->move(targetRightVel);
        } else {
            // For backwards motion, swap and negate motor commands
            drivetrain.leftMotors->move(-targetRightVel);
            drivetrain.rightMotors->move(-targetLeftVel);
        }

        pros::delay(10);
    }
     
    // Stop the robot at the end of the motion
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}