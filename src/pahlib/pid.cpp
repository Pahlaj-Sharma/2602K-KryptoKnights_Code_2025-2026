#include "pahlib/pid.hpp"
#include "pahlib/util.hpp"

namespace pahlib {
PID::PID(float kP, float kI, float kD, float kF, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

float PID::update(const float error, const float feedforward) {
    // calculate integral
    integral += error;
    if (sgn(error) != sgn((prevError)) && signFlipReset) integral = 0;
    if (std::fabs(error) > windupRange && windupRange != 0) integral = 0;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD + feedforward;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}
} // namespace pahlib