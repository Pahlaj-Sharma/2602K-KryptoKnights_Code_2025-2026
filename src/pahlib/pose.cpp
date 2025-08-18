#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "pahlib/pose.hpp"

pahlib::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

pahlib::Pose pahlib::Pose::operator+(const pahlib::Pose& other) const {
    return pahlib::Pose(this->x + other.x, this->y + other.y, this->theta);
}

pahlib::Pose pahlib::Pose::operator-(const pahlib::Pose& other) const {
    return pahlib::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float pahlib::Pose::operator*(const pahlib::Pose& other) const { return this->x * other.x + this->y * other.y; }

pahlib::Pose pahlib::Pose::operator*(const float& other) const {
    return pahlib::Pose(this->x * other, this->y * other, this->theta);
}

pahlib::Pose pahlib::Pose::operator/(const float& other) const {
    return pahlib::Pose(this->x / other, this->y / other, this->theta);
}

pahlib::Pose pahlib::Pose::lerp(pahlib::Pose other, float t) const {
    return pahlib::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float pahlib::Pose::distance(pahlib::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float pahlib::Pose::angle(pahlib::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

pahlib::Pose pahlib::Pose::rotate(float angle) const {
    return pahlib::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string pahlib::format_as(const pahlib::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("pahlib::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}
