#pragma once

#include <cmath>
#include <vector>
#include "pahlib/chassis/chassis.hpp"
#include "pahlib/pose.hpp"

namespace pahlib {
/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0

 * @return float - the limited value
 *
 * @b Example
 * @code {.cpp}
 * float limited = slew(100, // target value
 *                      0, // current value
 *                      10); // maximum allowed change
 * // limited == 10
 * float limited2 = slew(4, // target value
 *                       0, // current value
 *                       10); // maximum allowed change
 * // limited2 == 4
 * @endcode
 */
float slew(float target, float current, float maxChange);

/**
 * @brief Convert radians to degrees
 *
 * @param rad radians
 * @return float degrees
 *
 * @b Example
 * @code {.cpp}
 * radToDeg(M_PI); // returns 180
 * @endcode
 */
constexpr float radToDeg(float rad) { return rad * 180 / M_PI; }

/**
 * @brief Convert degrees to radians
 *
 * @param deg degrees
 * @return float radians
 *
 * @b Example
 * @code {.cpp}
 * degToRad(180); // returns 3.14159... (pi)
 * @endcode
 */
constexpr float degToRad(float deg) { return deg * M_PI / 180; }

/**
 * @brief Sanitize an angle so its positive and within the range of 0 to 2pi or 0 to 360
 *
 * @param angle the angle to sanitize
 * @param radians whether the angle is in radians or no. True by default
 * @return constexpr float
 *
 * @b Example
 * @code {.cpp}
 * // sanitize angle in degrees
 * sanitizeAngle(-90, false); // returns 270
 * sanitizeAngle(370, false); // returns 10
 * // sanitize angle in radians
 * sanitizeAngle(-M_PI, true); // returns pi
 * sanitizeAngle(7 * M_PI, true); // returns pi
 * // you can also use the default value of radians
 * sanitizeAngle(-M_PI); // returns pi
 * sanitizeAngle(7 * M_PI); // returns pi
 * @endcode
 */
constexpr float sanitizeAngle(float angle, bool radians = true) {
        if (radians) return std::fmod(std::fmod(angle, M_TWOPI) + M_TWOPI, M_TWOPI);
        else return std::fmod(std::fmod(angle, 360) + 360, 360);
    }

/**
 * @brief Calculate the error between 2 angles. Useful when calculating the error between 2 headings
 *
 * @param target target angle
 * @param position position angle
 * @param radians true if angle is in radians, false if not. False by default
 * @param direction which direction to turn to get to the target angle
 * @return float wrapped angle
 *
 * @b Example
 * @code {.cpp}
 * angleError(10, 350, false); // returns 20
 * angleError(350, 10, false); // returns -20
 * @endcode
 */
float angleError(float target, float position, bool radians = true,
                 AngularDirection direction = AngularDirection::AUTO);

/**
 * @brief Return the sign of a number
 *
 * @param x the number to get the sign of
 * @return int - -1 if negative, 1 if positive
 *
 * @b Example
 * @code {.cpp}
 * sgn(-10); // returns -1
 * sgn(10); // returns 1
 * sgn(0); // returns 1 (by convention)
 * @endcode
 */
template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }

/**
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return float
 *
 * @b Example
 * @code {.cpp}
 * std::vector<float> values = {1, 2, 3, 4, 5};
 * avg(values); // returns 3
 * @endcode
 */
float avg(std::vector<float> values);

/**
 * @brief Exponential moving average
 *
 * @param current current measurement
 * @param previous previous output
 * @param smooth smoothing factor (0-1). 1 means no smoothing, 0 means no change
 * @return float - the smoothed output
 *
 * @b Example
 * @code {.cpp}
 * ema(10, 0, 0.5); // returns 5
 * @endcode
 */
float ema(float current, float previous, float smooth);

/**
 * @brief Get the signed curvature of a circle that intersects the first pose and the second pose
 *
 * This is a very niche function that is only used in Pure Pursuit and Boomerang. It calculates the curvature of a
 * circle that is tangent to the first pose and intersects the second pose. It's also signed to indicate whether the
 * robot should turn clockwise or counter-clockwise to get to the second pose
 *
 * @note The circle will be tangent to the theta value of the first pose
 * @note The curvature is signed. Positive curvature means the circle is going clockwise, negative means
 * counter-clockwise
 * @note Theta has to be in radians and in standard form. That means 0 is right and increases counter-clockwise
 *
 * @param pose the first pose
 * @param other the second pose
 * @return float curvature
 *
 * @b Example
 * @code {.cpp}
 * Pose pose = {0, 0, 0};
 * Pose other = {0, 10, 0};
 * float curvature = getCurvature(pose, other);
 * @endcode
 */
float getCurvature(Pose pose, Pose other);

float lnrInterpolation(double x, double x1, double y1, double x2, double y2);

class Timer {
    public:
        /**
         * @brief Construct a new Timer
         *
         * @note the timer will start counting down as soon as it is created
         * @note the timer constructor is non-blocking so code after it will be executed immediately
         * @note if the timer is constructed in a global scope, its behavior is undefined. You can
         *       call set() before using the timer if you absolutely need to construct it in a global scope
         *
         * @param time how long to wait, in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * @endcode
         */
        Timer(uint32_t time);
        /**
         * @brief Get the amount of time the timer was set to
         *
         * @return uint32_t time, in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // get the time the timer was set to
         * const uint32_t time = timer.getTimeSet(); // time = 1000
         * @endcode
         */
        uint32_t getTimeSet();
        /**
         * @brief Get the amount of time left on the timer
         *
         * @return uint32_t time in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 300ms
         * pros::delay(300);
         * // get the time left on the timer
         * const uint32_t time = timer.getTimeLeft(); // time = 700
         * @endcode
         */
        uint32_t getTimeLeft();
        /**
         * @brief Get the amount of time passed on the timer
         *
         * @return uint32_t time in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 300ms
         * pros::delay(300);
         * // get the time passed on the timer
         * const uint32_t time = timer.getTimePassed(); // time = 300
         * @endcode
         */
        uint32_t getTimePassed();
        /**
         * @brief Get whether the timer is done or not
         *
         * @return true the timer is done
         * @return false the timer is not done
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = true
         * @endcode
         */
        bool isDone();
        /**
         * @brief Get whether the timer is paused or not
         *
         * @return true the timer is paused
         * @return false the timer is not paused
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // pause the timer
         * timer.pause();
         * // check if the timer is paused
         * bool paused = timer.isPaused(); // paused = true
         * // resume the timer
         * timer.resume();
         * // check if the timer is paused
         * paused = timer.isPaused(); // paused = false
         * @endcode
         */
        bool isPaused();
        /**
         * @brief Set the amount of time the timer should count down. Resets the timer
         *
         * @param time time in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // set the timer to wait for 2 seconds
         * timer.set(2000);
         * @endcode
         */
        void set(uint32_t time);
        /**
         * @brief reset the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 500ms
         * pros::delay(500);
         * // reset the timer
         * timer.reset();
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void reset();
        /**
         * @brief pause the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // pause the timer
         * timer.pause();
         * // delay for 2000ms
         * pros::delay(2000);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void pause();
        /**
         * @brief resume the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // pause the timer
         * timer.pause();
         * // delay for 500ms
         * pros::delay(500);
         * // resume the timer
         * timer.resume();
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void resume();
        /**
         * @brief wait until the timer is done
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // wait until the timer is done
         * timer.waitUntilDone();
         * std::cout << "done!" << std::endl;
         * @endcode
         */
        void waitUntilDone();
    private:
        uint32_t period;
        uint32_t lastTime;
        uint32_t timeWaited = 0;
        bool paused = false;
};

} // namespace pahlib
/**
 * @brief Get the data from a file in the format of a vector of poses
 *
 * @param path the path to the file
 * @return std::vector<pahlib::Pose> - the vector of poses
 *
 * @b Example
 * @code {.cpp}
 * asset path = "path/to/file.txt";
 * std::vector<pahlib::Pose> poses = getData(path);
 * @endcode
 */
std::vector<pahlib::Pose> getData(const asset& path);
/**
 * @brief Convert a string to a hex string
 *
 * @param input the input string
 * @return std::string - the hex string
 *
 * @b Example
 * @code {.cpp}
 * std::string hex = stringToHex("Hello");
 * // hex == "48656C6C6F"
 * @endcode
 */
std::string stringToHexRamsete(const std::string& input);
/**
 * @brief Read a string and split it by a delimiter
 *
 * @param input the input string
 * @param delimiter the delimiter to split by
 * @return std::vector<std::string> - the vector of strings
 *
 * @b Example
 * @code {.cpp}
 * std::string input = "Hello, World, How, Are, You";
 * std::vector<std::string> output = readElement(input, ", ");
 * // output == {"Hello", "World", "How", "Are", "You"}
 * @endcode
 */
std::vector<std::string> readElement(const std::string& input, const std::string& delimiter);