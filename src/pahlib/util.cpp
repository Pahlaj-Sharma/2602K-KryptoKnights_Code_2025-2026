#include "pahlib/pose.hpp"
#include "pahlib/util.hpp"
#include "pros/rtos.hpp"

float pahlib::slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

float pahlib::angleError(float target, float position, bool radians, AngularDirection direction) {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizeAngle(target, radians);
    position = sanitizeAngle(position, radians);
    const float max = radians ? M_TWOPI : 360;
    const float rawError = target - position;
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE: // turn clockwise
            return rawError < 0 ? rawError + max : rawError; // add max if sign does not match
        case AngularDirection::CCW_COUNTERCLOCKWISE: // turn counter-clockwise
            return rawError > 0 ? rawError - max : rawError; // subtract max if sign does not match
        default: // choose the shortest path
            return std::remainder(rawError, max);
    }
}

float pahlib::avg(std::vector<float> values) {
    float sum = 0;
    for (float value : values) { sum += value; }
    return sum / values.size();
}

float pahlib::ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}

float pahlib::getCurvature(Pose pose, Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    float side = pahlib::sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

float pahlib::lnrInterpolation(double x, double x1, double y1, double x2, double y2) {
    
    // Check for division by zero (if x1 equals x2)
    if (x1 == x2) return y1;
    // y = y1 + ( (x - x1) / (x2 - x1) ) * (y2 - y1)
    float slope = (y2 - y1) / (x2 - x1);
    float interpolated_y = y1 + slope * (x - x1);
    
    return interpolated_y;
}

std::vector<pahlib::Pose> getData(const asset& path) {
    std::vector<pahlib::Pose> robotPath;

    // format data from the asset
    const std::string data(reinterpret_cast<char*>(path.buf), path.size);
    const std::vector<std::string> dataLines = readElement(data, "\n");

    // read the points until 'endData' is read
    for (std::string line : dataLines) {
        if (line == "endData" || line == "endData\r") break;
        const std::vector<std::string> pointInput = readElement(line, ", "); // parse line
        // check if the line was read correctly
        if (pointInput.size() != 3) {
            break;
        }
        pahlib::Pose pathPoint(0, 0);
        pathPoint.x = std::stof(pointInput.at(0)); // x position
        pathPoint.y = std::stof(pointInput.at(1)); // y position
        pathPoint.theta = std::stof(pointInput.at(2)); // velocity
        robotPath.push_back(pathPoint); // save data
    }

    return robotPath;
}

std::string stringToHex(const std::string& input) {
    static const char hex_digits[] = "0123456789ABCDEF";

    std::string output;
    output.reserve(input.length() * 2);
    for (unsigned char c : input) {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
    }
    return output;
}

std::vector<std::string> readElement(const std::string& input, const std::string& delimiter) {
    std::string token;
    std::string s = input;
    std::vector<std::string> output;
    size_t pos = 0;

    // main loop
    while ((pos = s.find(delimiter)) != std::string::npos) { // while there are still delimiters in the string
        token = s.substr(0, pos); // processed substring
        output.push_back(token);
        s.erase(0, pos + delimiter.length()); // remove the read substring
    }

    output.push_back(s); // add the last element to the returned string

    return output;
}

using namespace pahlib;

Timer::Timer(uint32_t time)
    : period(time) {
    lastTime = pros::millis();
}

uint32_t Timer::getTimeSet() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    return period;
}

uint32_t Timer::getTimeLeft() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    const int delta = period - timeWaited; // calculate how much time is left
    return (delta > 0) ? delta : 0; // return 0 if timer is done
}

uint32_t Timer::getTimePassed() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time;
    return timeWaited;
}

bool Timer::isDone() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    const int delta = period - timeWaited; // calculate how much time is left
    return delta <= 0;
}

bool Timer::isPaused() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    return paused;
}

void Timer::set(uint32_t time) {
    period = time; // set how long to wait
    reset();
}

void Timer::reset() {
    timeWaited = 0;
    lastTime = pros::millis();
}

void Timer::pause() {
    if (!paused) lastTime = pros::millis();
    paused = true;
}

void Timer::resume() {
    if (paused) lastTime = pros::millis();
    paused = false;
}

void Timer::waitUntilDone() {
    do pros::delay(5);
    while (!this->isDone());
}
