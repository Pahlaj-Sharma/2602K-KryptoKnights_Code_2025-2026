#include "main.h"
#include "autons.hpp"
#include "robot_config.hpp"
#include "lemlib/api.hpp"
#include <cmath>



void auton1() {
    
}
void auton2() {
    
}
void auton3() {
    
}
void auton4() {
    
}
void auton5() {
    
}
void auton6() {
    
}
void auton7() {
    
}
void auton8() {
    
}
void auton9() {
    
}
void auton10() {
    
}

void moveLinear(double inches, int timeout) {
        // Get the robot's current pose (X, Y, Heading)
        lemlib::Pose currentPose = chassis.getPose();

        // Convert current heading from degrees to radians for sin/cos functions
        // LemLib's getPose().theta is in degrees
        double currentThetaRad = lemlib::degToRad(currentPose.theta);

        // Calculate the target X and Y coordinates based on current heading and desired distance
        double targetX = currentPose.x + (inches * std::cos(currentThetaRad));
        double targetY = currentPose.y + (inches * std::sin(currentThetaRad));

        // Move to the calculated target pose, maintaining the current heading (currentPose.theta)
        chassis.moveToPose(targetX, targetY, currentPose.theta, timeout);
    }

void resetOdometry() {
        lemlib::Pose currentPose = chassis.getPose();
        double currentTheta = currentPose.theta; // Theta is accurate, so we keep it

        // Variables to store the new calculated X and Y, initially set to current
        double newX = currentPose.x;
        double newY = currentPose.y;

        // Read all distance sensor values
        double distFront = frontDistance.get() * 0.04;
        double distBack = backDistance.get() * 0.04;
        double distLeft = leftDistance.get() * 0.04;
        double distRight = rightDistance.get() * 0.04;

        // --- Determine Reset Points based on Detected Walls and Current Heading ---
        // This logic assumes the robot is generally perpendicular to walls when resetting.
        // LemLib's angleError function is useful for checking if an angle is "close enough" to a cardinal direction.
        // lemlib::angleError(target_angle, current_angle) returns smallest difference in degrees.

        bool frontWallDetected = distFront < WALL_DETECTION_THRESHOLD_INCHES;
        bool backWallDetected = distBack < WALL_DETECTION_THRESHOLD_INCHES;
        bool leftWallDetected = distLeft < WALL_DETECTION_THRESHOLD_INCHES;
        bool rightWallDetected = distRight < WALL_DETECTION_THRESHOLD_INCHES;

        // --- Reset X-Coordinate ---
        // If robot is generally facing East (0 deg) or West (180 deg)
        if (std::fabs(lemlib::angleError(currentTheta, 0)) < 15 || std::fabs(lemlib::angleError(currentTheta, 180)) < 15) {
            if (frontWallDetected && std::fabs(lemlib::angleError(currentTheta, 0)) < 15) {
                // Facing positive X wall (East side of field)
                newX = FIELD_HALF_LENGTH_INCHES - (distFront + DS_FRONT_OFFSET_INCHES + ROBOT_HALF_LENGTH_INCHES);
                pros::lcd::print(7, "Resetting X: Front-East Wall");
            } else if (backWallDetected && std::fabs(lemlib::angleError(currentTheta, 180)) < 15) {
                // Facing negative X wall (West side of field)
                newX = -FIELD_HALF_LENGTH_INCHES + (distBack + DS_BACK_OFFSET_INCHES + ROBOT_HALF_LENGTH_INCHES);
                pros::lcd::print(7, "Resetting X: Back-West Wall");
            }
        }
        // If robot is generally facing North (90 deg) or South (270 deg)
        else if (std::fabs(lemlib::angleError(currentTheta, 90)) < 15 || std::fabs(lemlib::angleError(currentTheta, 270)) < 15) {
            if (rightWallDetected && std::fabs(lemlib::angleError(currentTheta, 90)) < 15) {
                // Right sensor detects wall, facing North, so right sensor is facing +X wall
                // (This implies the side sensors are positioned to measure to the field's X boundaries)
                newX = FIELD_HALF_LENGTH_INCHES - (distRight + DS_RIGHT_OFFSET_INCHES + ROBOT_HALF_WIDTH_INCHES);
                pros::lcd::print(7, "Resetting X: Right-North Wall");
            } else if (leftWallDetected && std::fabs(lemlib::angleError(currentTheta, 270)) < 15) {
                // Left sensor detects wall, facing South, so left sensor is facing -X wall
                newX = -FIELD_HALF_LENGTH_INCHES + (distLeft + DS_LEFT_OFFSET_INCHES + ROBOT_HALF_WIDTH_INCHES);
                pros::lcd::print(7, "Resetting X: Left-South Wall");
            }
        }
        
        // --- Reset Y-Coordinate ---
        // If robot is generally facing North (90 deg) or South (270 deg)
        if (std::fabs(lemlib::angleError(currentTheta, 90)) < 15 || std::fabs(lemlib::angleError(currentTheta, 270)) < 15) {
            if (frontWallDetected && std::fabs(lemlib::angleError(currentTheta, 90)) < 15) {
                // Facing positive Y wall (North side of field)
                newY = FIELD_HALF_WIDTH_INCHES - (distFront + DS_FRONT_OFFSET_INCHES + ROBOT_HALF_LENGTH_INCHES);
                pros::lcd::print(7, "Resetting Y: Front-North Wall");
            } else if (backWallDetected && std::fabs(lemlib::angleError(currentTheta, 270)) < 15) {
                // Facing negative Y wall (South side of field)
                newY = -FIELD_HALF_WIDTH_INCHES + (distBack + DS_BACK_OFFSET_INCHES + ROBOT_HALF_LENGTH_INCHES);
                pros::lcd::print(7, "Resetting Y: Back-South Wall");
            }
        }
        // If robot is generally facing East (0 deg) or West (180 deg)
        else if (std::fabs(lemlib::angleError(currentTheta, 0)) < 15 || std::fabs(lemlib::angleError(currentTheta, 180)) < 15) {
             if (rightWallDetected && std::fabs(lemlib::angleError(currentTheta, 0)) < 15) {
                // Right sensor detects wall, facing East, so right sensor is facing -Y wall
                newY = -FIELD_HALF_WIDTH_INCHES + (distRight + DS_RIGHT_OFFSET_INCHES + ROBOT_HALF_WIDTH_INCHES);
                pros::lcd::print(7, "Resetting Y: Right-East Wall");
            } else if (leftWallDetected && std::fabs(lemlib::angleError(currentTheta, 180)) < 15) {
                // Left sensor detects wall, facing West, so left sensor is facing +Y wall
                newY = FIELD_HALF_WIDTH_INCHES - (distLeft + DS_LEFT_OFFSET_INCHES + ROBOT_HALF_WIDTH_INCHES);
                pros::lcd::print(7, "Resetting Y: Left-West Wall");
            }
        }

        // --- Corner Reset (More Accurate) ---
        // If two perpendicular sensors detect walls and robot is aligned to a corner
        // Example: Front sensor detecting +X wall, Right sensor detecting -Y wall, and theta is 0 degrees.
        if (frontWallDetected && rightWallDetected && std::fabs(lemlib::angleError(currentTheta, 0)) < 15) {
            // Assuming robot is in bottom-right corner (positive X, negative Y) and facing 0 degrees
            newX = FIELD_HALF_LENGTH_INCHES - (distFront + DS_FRONT_OFFSET_INCHES + ROBOT_HALF_LENGTH_INCHES);
            newY = -FIELD_HALF_WIDTH_INCHES + (distRight + DS_RIGHT_OFFSET_INCHES + ROBOT_HALF_WIDTH_INCHES);
            pros::lcd::print(7, "Resetting: Bottom-Right Corner");
        }
        // Add more corner cases as needed (e.g., front-left, back-right, back-left)

        // Set the new odometry pose, keeping the current accurate theta
        chassis.setPose(newX, newY, currentTheta);
        pros::lcd::print(1, "Odometry Reset to X: %.2f, Y: %.2f", newX, newY);
    }