#include "pahlib/chassis/chassis.hpp"
#include "pahlib/util.hpp"
#include "pahlib/timer.hpp"

/**
 * @brief Finds the closest point on the path to the robot.
 *
 * @param pose The current pose of the robot.
 * @param path The path to follow.
 * @param lastIndex The index of the last closest point.
 * @return A pair containing the index of the closest segment and the projection parameter 't'.
 */
std::pair<int, float> findClosestPoint(const pahlib::Pose& pose, const std::vector<pahlib::Pose>& path, int lastIndex) {
    float minDistance = std::numeric_limits<float>::max();
    int closestSegment = lastIndex;
    float closestT = 0;

    // Number of segments to check ahead of the last closest point
    const int lookahead = 3;

    for (size_t i = lastIndex; i < std::min((size_t)lastIndex + lookahead, path.size() - 1); ++i) {
        const pahlib::Pose& startPoint = path[i];
        const pahlib::Pose& endPoint = path[i + 1];

        const float dx = endPoint.x - startPoint.x;
        const float dy = endPoint.y - startPoint.y;
        const float segmentLengthSquared = dx * dx + dy * dy;

        if (segmentLengthSquared < 0.0001f) continue;

        const float robotToStart_x = pose.x - startPoint.x;
        const float robotToStart_y = pose.y - startPoint.y;

        // Project the robot's position onto the path segment
        float t = (robotToStart_x * dx + robotToStart_y * dy) / segmentLengthSquared;
        t = std::max(0.0f, std::min(1.0f, t)); // Clamp t to the range [0, 1]

        const float projectedX = startPoint.x + t * dx;
        const float projectedY = startPoint.y + t * dy;

        const float distance = std::hypot(pose.x - projectedX, pose.y - projectedY);

        if (distance < minDistance) {
            minDistance = distance;
            closestSegment = i;
            closestT = t;
        }
    }

    return {closestSegment, closestT};
}

struct PursuitError {
    float crossTrackError;
    float currentHeadingError;
    float futureHeadingError;
};

/**
 * @brief Calculates the cross-track error and heading error for the Stanley controller.
 *
 * @param pose The current robot pose.
 * @param path The path to follow.
 * @param i The index of the current path segment.
 * @param t The projection parameter along the segment.
 * @param lookahead The lookahead distance.
 * @return A PursuitError struct containing the calculated errors.
 */
PursuitError calculateErrors(const pahlib::Pose& pose, const std::vector<pahlib::Pose>& path, int i, float t,
                             float lookahead) {
    const auto& p1 = path[i];
    const auto& p2 = path[i + 1];

    // Calculate the heading of the current path segment
    const float currentHeading = std::atan2(p2.x - p1.x, p2.y - p1.y);

    // Find the future heading by looking ahead on the path
    float remaining = lookahead;
    int idx = i;
    float futureHeading = currentHeading;
    while (idx < path.size() - 1 && remaining > 0) {
        const float seg = std::hypot(path[idx + 1].x - path[idx].x, path[idx + 1].y - path[idx].y);
        if (seg > remaining) {
            futureHeading = std::atan2(path[idx + 1].x - path[idx].x, path[idx + 1].y - path[idx].y);
            break;
        }
        remaining -= seg;
        ++idx;
    }

    // Calculate the cross-track error
    const float projX = p1.x + t * (p2.x - p1.x);
    const float projY = p1.y + t * (p2.y - p1.y);
    const float dx = pose.x - projX;
    const float dy = pose.y - projY;
    // The sign of the cross-track error is determined by the sign of the dot product
    // between the vector to the robot and a vector perpendicular to the path
    const float crossTrackError =
        std::copysign(std::hypot(dx, dy), std::cos(currentHeading) * dx - std::sin(currentHeading) * dy);

    const float currentHeadingError = pahlib::angleError(pose.theta, currentHeading);
    const float futureHeadingError = pahlib::angleError(pose.theta, futureHeading);

    return {crossTrackError, currentHeadingError, futureHeadingError};
}

void pahlib::Chassis::stanley(const asset& path, float lookahead, float stanleyGain, float headingFF, int timeout, 
                              bool forwards, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;

    if (async) {
        pros::Task task([&]() { stanley(path, lookahead, timeout, forwards, false, stanleyGain, headingFF); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    // Reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    std::vector<pahlib::Pose> pathPoints = getData(path);
    if (pathPoints.size() < 2) {
        distTraveled = -1;
        this->endMotion();
        return;
    }

    Pose pose = this->getPose(true);
    Pose lastPose = pose;
    float prevVel = 0;
    int lastIndex = 0;
    const int compState = pros::competition::get_status();
    distTraveled = 0;

    const float k = stanleyGain;
    const pahlib::Pose& finalPoint = pathPoints.back();
    pahlib::Timer timer(timeout);

    // Main control loop
    while (this->motionRunning && !timer.isDone() && pros::competition::get_status() == compState) {
        pose = this->getPose(true);
        if (!forwards) pose.theta -= M_PI;

        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // Find the closest point on the path
        auto [closestSegment, t] = findClosestPoint(pose, pathPoints, lastIndex);
        lastIndex = closestSegment;

        // Update exit conditions
        const float currentDistToTarget = std::hypot(pose.x - finalPoint.x, pose.y - finalPoint.y);
        if (lateralSmallExit.update(currentDistToTarget) || lateralLargeExit.update(currentDistToTarget)) break;

        // Calculate errors for the Stanley controller
        const auto errors = calculateErrors(pose, pathPoints, closestSegment, t, lookahead);

        // Get the target velocity for the current segment
        float targetVel = pathPoints[closestSegment].theta;
        targetVel = slew(targetVel, prevVel, lateralSettings.slew);
        prevVel = targetVel;

        // Stanley steering control law
        const float blendedHeadingError = (1 - headingFF) * errors.currentHeadingError + headingFF * errors.futureHeadingError;
        const float stanleyTerm = std::atan2(k * errors.crossTrackError, targetVel);
        const float steeringAngle = blendedHeadingError + stanleyTerm;

        // Convert target velocity and steering angle to wheel velocities
        const float throttle = targetVel * std::cos(steeringAngle);
        const float turn = (targetVel * std::sin(steeringAngle)) / (drivetrain.trackWidth / 2.0);

        float targetLeftVel = throttle - turn * drivetrain.trackWidth / 2.0;
        float targetRightVel = throttle + turn * drivetrain.trackWidth / 2.0;

        // Scale wheel velocities to fit within the motor limits
        const float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127.0f;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }

        // Move the motors
        if (forwards) {
            drivetrain.leftMotors->move(targetLeftVel);
            drivetrain.rightMotors->move(targetRightVel);
        } else {
            drivetrain.leftMotors->move(-targetRightVel);
            drivetrain.rightMotors->move(-targetLeftVel);
        }

        pros::delay(10);
    }

    // Stop the motors
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}