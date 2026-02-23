// RclTracking.cpp
// Description: This file implements the RCL Tracking system for a robot, including
//              sensor management, obstacle detection, position tracking and syncing
//              using VEX V5 distance sensors.
//              Most functionalities are achieved through basic sensor fusion and
//              intersection math.

#include "pahlib/RclTracking.hpp"
#include <cmath>

inline double roundTwoPlaces(double x) {
    return std::round(x*100)/100;
}

// Line obstacle class
Singly_Linked_List<Line_Obstacle> Line_Obstacle::obstacleCollection = Singly_Linked_List<Line_Obstacle>();
Line_Obstacle::Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs)
    : lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    // Init line
    line.pt1[0] = x1;
    line.pt1[1] = y1;
    line.pt2[0] = x2;
    line.pt2[1] = y2;
    line.slope = (y2 - y1) / (x2 - x1);
    line.yIntercept = y1 - line.slope * x1;
    // Add to collection if space available
    Line_Obstacle::obstacleCollection.add_front(this);
}

// Check if the obstacle has expired
bool Line_Obstacle::expired() {
    return lifeTimer.timeIsUp();
}

// Check if sensor ray intersects this obstacle line
bool Line_Obstacle::isIntersecting(const SensorPose& sp) const {
    // Ray direction vector
    double angRad = degToRad(botToTrig(sp.heading));
    double vAx = std::cos(angRad);
    double vAy = std::sin(angRad);

    // Obstacle segment vector
    double vBx = line.pt2[0] - line.pt1[0];
    double vBy = line.pt2[1] - line.pt1[1];

    // Determinant (Cross product of direction vectors)
    double det = (vAx * vBy) - (vAy * vBx);

    // If det is 0, the ray and the obstacle are parallel
    if (std::abs(det) < 1e-6) return false;

    // Solve for t (distance along sensor ray) and u (position along obstacle segment)
    // Formula: P_sensor + t*vA = P_obstacle_start + u*vB
    double dx = line.pt1[0] - sp.x;
    double dy = line.pt1[1] - sp.y;

    double t = (dx * vBy - dy * vBx) / det;
    double u = (dx * vAy - dy * vAx) / det;

    // Valid intersection if:
    // 1. t > 0 (Intersection is in front of the sensor)
    // 2. 0 <= u <= 1 (Intersection lies on the actual segment, not just the infinite line)
    return (t > 0 && u >= 0 && u <= 1);
}

// Polygon obstacles
void Line_Obstacle::addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs) {
    // If the points are not enough to form a polygon, do nothing
    if (points.size() < 3) return;
    // Create a line for each pair of points
    for (size_t i = 0; i < points.size(); ++i) {
        size_t next = (i + 1) % points.size();
        new Line_Obstacle(points[i].first, points[i].second, points[next].first, points[next].second, lifeTimeMs);
    }
}

// Circle obstacle class
Singly_Linked_List<Circle_Obstacle> Circle_Obstacle::obstacleCollection = Singly_Linked_List<Circle_Obstacle>();
Circle_Obstacle::Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs)
    : x(x_), y(y_), radius(r_), lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    Circle_Obstacle::obstacleCollection.add_front(this);
}

// Check if the obstacle expired
bool Circle_Obstacle::expired() {
    return lifeTimer.timeIsUp();
}

// Check if sensor ray intersects this obstacle circle
bool Circle_Obstacle::isIntersecting(const SensorPose& sp) const {
    double angRad = degToRad(botToTrig(sp.heading));
    double vx = std::cos(angRad);
    double vy = std::sin(angRad);

    // Vector from sensor to circle center
    double dx = x - sp.x;
    double dy = y - sp.y;

    // Calculate 't' (the distance along the ray to the point closest to the circle center)
    // t = dot product of (vector to center) and (ray direction)
    double t = dx * vx + dy * vy;

    // If t is negative, the circle is behind the sensor
    if (t < 0) return false;

    // Find the point on the ray at distance t
    double closestX = sp.x + t * vx;
    double closestY = sp.y + t * vy;

    // Check distance from that point to circle center
    double distSq = std::pow(closestX - x, 2) + std::pow(closestY - y, 2);
    
    return distSq <= (radius * radius);
}

// RCL sensor class
RclSensor::RclSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol)
    : sensor(distSensor), mainAngle(mainAng), angleTolerance(std::abs(angleTol)) {
    offsetDist = std::hypot(horizOffset, vertOffset);
    offsetAngle = std::fmod(((std::atan2(vertOffset, horizOffset)*180.0/M_PI) + 360), 360);
    RclSensor::sensorCollection.push_back(this);
}

// Compute pose of the ray origin & slope
void RclSensor::updatePose(const pahlib::Pose& botPose) {
    // sensor's angle position relative to the bot's center
    double theta = degToRad(offsetAngle - botPose.theta);
    sp.x = botPose.x + std::cos(theta) * offsetDist;
    sp.y = botPose.y + std::sin(theta) * offsetDist;
    // sensor ray's heading
    sp.heading = std::fmod(botPose.theta + mainAngle, 360.0);
    if (sp.heading <= 0) sp.heading += 360.0;  // avoid 0 or negative
    // slope and y-intercept
    sp.slope = std::tan(degToRad(botToTrig(sp.heading)));
    sp.yIntercept = sp.y - sp.slope * sp.x;
}

bool RclSensor::isValid(double distVal) const {
    if (distVal > 2000) return false;  // Invalid Distance
    if (distVal > 200 && this->sensor->get_confidence() < 60) return false; // Invalid confidence
    if (std::abs( std::fmod(this->sp.heading, 90.0) ) > angleTolerance &&
        std::abs( std::fmod(this->sp.heading, 90.0) ) < (90 - angleTolerance)) return false;   // Limit heading to intervals around 90 degrees to increase accuracy
    // Detect if the ray is intersecting with any circular obstacles
    for (auto* item : Circle_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }
    // Detect if the ray is intersectign with any line obstacles
    for (auto* item : Line_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }

    return true;
}

// Return which coordinate (X or Y) and its value
std::pair<CoordType, double> RclSensor::getBotCoord(const pahlib::Pose& botPose, double accum) {

    // Update sensor pose
    this->updatePose(botPose);

    // accumulative?
    double val = std::isnan(accum) ? sensor->get() : accum;
    
    // verify sensor data
    if (!isValid(val)) return {CoordType::INVALID, 0.0};
    val *= mmToInch;

    double angRad = degToRad(botToTrig(this->sp.heading));
    double cosA = std::cos(angRad);
    double sinA = std::sin(angRad);

    double minDist = 1e9; // Start with a huge number
    int wall = -1;

    // Check X-walls (Vertical lines)
    if (std::abs(cosA) > 1e-6) {
        double dEast = (FIELD_HALF_LENGTH - sp.x) / cosA;
        if (dEast > 0 && dEast < minDist) { minDist = dEast; wall = 2; }
        
        double dWest = (FIELD_NEG_HALF_LENGTH - sp.x) / cosA;
        if (dWest > 0 && dWest < minDist) { minDist = dWest; wall = 4; }
    }

    // Check Y-walls (Horizontal lines)
    if (std::abs(sinA) > 1e-6) {
        double dNorth = (FIELD_HALF_LENGTH - sp.y) / sinA;
        if (dNorth > 0 && dNorth < minDist) { minDist = dNorth; wall = 1; }
        
        double dSouth = (FIELD_NEG_HALF_LENGTH - sp.y) / sinA;
        if (dSouth > 0 && dSouth < minDist) { minDist = dSouth; wall = 3; }
    }

    // Compute coordinate
    double res;
    CoordType type;
    if (wall == 1) { type = CoordType::Y; res = FIELD_HALF_LENGTH - sinA * val; }
    else if (wall == 2) { type = CoordType::X; res = FIELD_HALF_LENGTH - cosA * val; }
    else if (wall == 3) { type = CoordType::Y; res = FIELD_NEG_HALF_LENGTH - sinA * val; }
    else if (wall == 4) { type = CoordType::X; res = FIELD_NEG_HALF_LENGTH - cosA * val; }
    else return {CoordType::INVALID, 0.0};  // Error case

    // Adjust by offset
    double offRad = degToRad(offsetAngle - botPose.theta);
    if (type == CoordType::X) res -= std::cos(offRad) * offsetDist;
    else if (type == CoordType::Y) res -= std::sin(offRad) * offsetDist;

    return {type, res};
}

int RclSensor::rawReading() const { return sensor->get(); }
SensorPose RclSensor::getPose() const { return sp; }

// Main RCL Tracking
RclTracking::RclTracking(pahlib::Chassis* chassis_,
            int frequencyHz_,
            bool autoSync_,
            double minDelta_,
            double maxDelta_,
            double maxDeltaFrompahlib_,
            double maxSyncPerSec_,
            int minPause_)
    : chassis(chassis_),
        goalMSPT(std::round(1000.0 / frequencyHz_)),
        minPause(minPause_),
        maxSyncPT(maxSyncPerSec_ / frequencyHz_),
        minDelta(minDelta_),
        maxDelta(maxDelta_),
        maxDeltaFrompahlib(maxDeltaFrompahlib_),
        autoSync(autoSync_),
        latestPrecise{0, 0, 0},
        poseAtLatest{0, 0, 0} {}

// Start background task
void RclTracking::startTracking() {
    // Start position update loop
    if (mainLoopTask == nullptr)  {
        mainLoopTask = new pros::Task([this](){ this->mainLoop(); });
    }
    // Start miscellaneous update loop
    if (miscLoopTask == nullptr) {
        miscLoopTask = new pros::Task([this](){ this->miscLoop(); });
    }
}
void RclTracking::stopTracking() {
    // Stop both loops if possible
    if (mainLoopTask != nullptr) { mainLoopTask->remove(); delete mainLoopTask; mainLoopTask = nullptr; }
    if (miscLoopTask != nullptr) { miscLoopTask->remove(); delete miscLoopTask; miscLoopTask = nullptr; } 
}
// Accessors
pahlib::Pose RclTracking::getRclPose() const {
    auto chassisPose = chassis->getPose();
    return { latestPrecise.x + (chassisPose.x - poseAtLatest.x),
                latestPrecise.y + (chassisPose.y - poseAtLatest.y),
                chassisPose.theta };
}
void RclTracking::setRclPose(const pahlib::Pose& p) {
    latestPrecise = p;
    poseAtLatest = chassis->getPose();
}
void RclTracking::updateBotPose() {
    auto p = getRclPose();
    chassis->setPose(p);
    setRclPose(p);
}
void RclTracking::updateBotPose(RclSensor* sens) {
    if (sens != nullptr && chassis != nullptr) {
        // Get data from the target sensor
        auto data = sens->getBotCoord(chassis->getPose());
        // Retrieve pahlib pose as the base
        auto pose = chassis->getPose();

        // Update pose based on sensor reading
        if (data.first == CoordType::X) pose.x = data.second;
        else if (data.first == CoordType::Y) pose.y = data.second;
        else return;

        // Sync to pahlib
        chassis->setPose({pose.x, pose.y, chassis->getPose().theta});
        setRclPose({pose.x, pose.y, chassis->getPose().theta});
    }
}

// Accumulation control
void RclTracking::startAccumulating(bool autoUpdateAfterAccum) { accumulating = true; updateAfterAccum = autoUpdateAfterAccum; }
void RclTracking::stopAccumulating() { accumulating = false; }
void RclTracking::accumulateFor (int ms, bool autoUpdateAfterAccum) {
    startAccumulating( autoUpdateAfterAccum );
    Timer t(ms);
    while (!t.timeIsUp()) { pros::delay(minPause); }
    stopAccumulating();
}

// Reset Rcl
void RclTracking::discardData () {
    latestPrecise = chassis->getPose();
    poseAtLatest = chassis->getPose();
}

void RclTracking::setMaxSyncPerSec(double maxSyncPerSec_) {
    maxSyncPT = maxSyncPerSec_ / (1000.0 / goalMSPT);
};

// Single updates
void RclTracking::mainUpdate() {
    // Verify that there is at least one sensor
    if (RclSensor::sensorCollection.size() > 0) {

        // Accumulators
        std::vector<int> accTotal(RclSensor::sensorCollection.size());
        std::vector<int> accCount(RclSensor::sensorCollection.size());
        // If accumulating, gather readings
        while (accumulating) {
            for (int i = 0; i < RclSensor::sensorCollection.size(); i++) {
                accTotal[i] += RclSensor::sensorCollection[i]->rawReading();
                accCount[i] ++;
            }
            pros::delay(goalMSPT);
        }

        // Collections
        std::vector<double> xs, ys;

        // Make sure RclPosition doesn't deviate too much from the chassis position
        auto botPose = getRclPose();
        double diff_from_pahlib = std::hypot(botPose.x-chassis->getPose().x, botPose.y-chassis->getPose().y);
        if (diff_from_pahlib > maxDeltaFrompahlib) {
            botPose.x += (chassis->getPose().x-botPose.x) / diff_from_pahlib;
            botPose.y += (chassis->getPose().y-botPose.y) / diff_from_pahlib;
        }

        // Loop sensors
        for (int i = 0; i < RclSensor::sensorCollection.size(); i++) {
            auto sens = RclSensor::sensorCollection[i];

            double avg = (accCount[i] > 0) ? (1.0 * accTotal[i] / accCount[i]) : NAN;
            auto [type, coord] = sens->getBotCoord(botPose, avg);

            // Validate and collect
            if (type == CoordType::X) {
                double diff = std::abs(coord - botPose.x);
                if (diff <= maxDelta) xs.push_back(coord);
            }
            else if (type == CoordType::Y) {
                double diff = std::abs(coord - botPose.y);
                if (diff <= maxDelta) ys.push_back(coord);
            }
        }

        // Update means
        if (!xs.empty()) {
            double meanX = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
            if (meanX > FIELD_NEG_HALF_LENGTH && meanX < FIELD_HALF_LENGTH && std::abs(meanX-botPose.x) >= minDelta) {
                latestPrecise.x = meanX;
                poseAtLatest.x = chassis->getPose().x;
            }
        }
        if (!ys.empty()) {
            double meanY = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
            if (meanY > FIELD_NEG_HALF_LENGTH && meanY < FIELD_HALF_LENGTH && std::abs(meanY-botPose.y) >= minDelta) {
                latestPrecise.y = meanY;
                poseAtLatest.y = chassis->getPose().y;
            }
        }

        // Determine if bot position should be automatically updated
        if (updateAfterAccum && std::any_of(accCount.begin(), accCount.end(), [](int c){ return c>0; }))
            updateBotPose();
    }
}
void RclTracking::syncUpdate() {
    // variables
    double x_diff = 0.0;
    double y_diff = 0.0;
    double real_diff = 0.0;

    double x_update = 0.0;
    double y_update = 0.0;

    // retrive rcl-based position
    pahlib::Pose currRclPosition = getRclPose();

    // calculate differences
    x_diff = currRclPosition.x - chassis->getPose().x;
    y_diff = currRclPosition.y - chassis->getPose().y;
    real_diff = std::hypot(std::abs(x_diff), std::abs(y_diff));

    // If within maximum sync distance, update the chassis pose directly
    if (real_diff <= maxSyncPT) {
        chassis->setPose(currRclPosition.x, currRclPosition.y, chassis->getPose().theta);
        
        poseAtLatest.x += x_diff;
        poseAtLatest.y += y_diff;
        poseAtLatest.theta = chassis->getPose().theta;
    }
    // Otherwise, only sync part of the discrepency
    else {
        // Calculate the updating amount
        x_update = x_diff / real_diff * maxSyncPT;
        y_update = y_diff / real_diff * maxSyncPT;

        // Update (Sync)
        chassis->setPose(chassis->getPose().x+x_update, chassis->getPose().y+y_update, chassis->getPose().theta);
        
        poseAtLatest.x += x_update;
        poseAtLatest.y += y_update;
        poseAtLatest.theta = chassis->getPose().theta;
    }
}
void RclTracking::lifeTimeUpdate() {
    // Clean circular obstacles
    auto circle_itr = Circle_Obstacle::obstacleCollection.begin();
    while (circle_itr != Circle_Obstacle::obstacleCollection.end()) {
        if ((**circle_itr).expired()) circle_itr.remove(true);
        else ++circle_itr;
    }

    // Clean line obstacles
    auto line_itr = Line_Obstacle::obstacleCollection.begin();
    while (line_itr != Line_Obstacle::obstacleCollection.end()) {
        if ((**line_itr).expired()) line_itr.remove(true);
        else ++line_itr;
    }
}

// Update loops
void RclTracking::mainLoop() {
    Timer frequencyTimer(goalMSPT);    // Create timer for frequency

    while (true) {
        // Reset frequency timer
        frequencyTimer.reset();

        // Call the update function
        mainUpdate();

        // Delay to save resources
        if ( frequencyTimer.timeLeft() < minPause ) pros::delay(minPause); // Ensure that the loop pauses at least for minPause
        else pros::delay(frequencyTimer.timeLeft()); // Otherwise, wait for the remaining time
    }
}
void RclTracking::miscLoop() {

    Timer frequencyTimer(goalMSPT);    // Create timer for frequency

    while (true) {

        frequencyTimer.reset(); // Reset frequency timer

        // main update functions
        if (autoSync) { syncUpdate(); }
        // lifeTimeUpdate();

        // Pause for the remaining time
        if (frequencyTimer.timeLeft() < minPause) pros::delay(minPause); // Ensure that the loop pauses at least for minPause
        else pros::delay(frequencyTimer.timeLeft()); // Otherwise, wait for the remaining time
    }
}