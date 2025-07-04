#include "main.h"         // PROS main header file, essential for all PROS projects.
#include "lemlib/api.hpp" // LemLib API for advanced odometry, chassis control, and motion profiling.
#include "pros/adi.hpp"   // PROS ADI (Analog/Digital Input) for connecting external sensors to ADI ports.
#include <map>            // Standard C++ library for using std::map containers (key-value pairs).
#include <string>         // Standard C++ library for using std::string objects.

// --- Controller Definition ---
// Initializes the primary VEX V5 controller connected to the robot.
// pros::E_CONTROLLER_MASTER specifies that this is the main driver controller.
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Motor Definitions ---
// MotorGroup is a LemLib class that allows you to control multiple motors as a single unit.
// The constructor takes an initializer list of motor port numbers and a gearset.
// Negative motor port numbers indicate that the motor's direction should be reversed.

// Right Motors: Grouping motors on ports 11, 13, 12 for the right side of the drivetrain.
// pros::MotorGearset::blue indicates they are configured with blue (600 RPM) cartridges.
pros::MotorGroup right_motors({11, 13, 12}, pros::MotorGearset::blue);

// Left Motors: Grouping motors on ports 14, 15, 16 for the left side of the drivetrain.
// These motors are explicitly reversed (-14, -15, -16) to ensure correct movement direction.
// Also configured with blue (600 RPM) cartridges.
pros::MotorGroup left_motors({-14, -15, -16}, pros::MotorGearset::blue);

// --- Sensor Definitions ---
// Inertial Sensor (IMU): Initializes an IMU on smart port 2.
// The IMU provides heading, rotation, and acceleration data, crucial for odometry.
pros::Imu imu(2);

// Horizontal Odometry Tracker: Initializes a PROS Rotation Sensor.
// Connected to ADI port -3 (the negative sign indicates the sensor's direction is reversed).
// This sensor typically tracks horizontal movement of the robot for odometry.
pros::Rotation horizontal_encoder(-3);

// Vertical Odometry Tracker: Initializes a PROS Rotation Sensor.
// Connected to ADI port 17.
// This sensor typically tracks vertical movement of the robot for odometry.
pros::Rotation vertical_encoder(17);

// Autonomous Selector Potentiometer: Initializes a potentiometer on ADI port 6.
// This analog sensor is used to select different autonomous routines based on its rotation.
pros::adi::Potentiometer autonSelector(6);

// Team Selector Digital Input: Initializes a potentiometer on ADI port 7.
// NOTE: Although declared as `pros::adi::Potentiometer`, your previous usage (e.g., `teamSelector.get_angle()`)
// suggests it's being used for a switch-like two-state selection (RED/BLUE) rather than a continuous range.
// If it's a simple digital switch, `pros::adi::DigitalIn` would be more appropriate.
// The error `error: cannot convert '<brace-enclosed initializer list>'` in `moveToPose` might be related
// to C++ standard issues affecting how parameters are passed to LemLib functions, not directly this line.
pros::adi::Potentiometer teamSelector(7);


// Distance Sensors (for potential odometry resets or object detection)
// All are currently on port 3. In a typical robot, these would be on different ports
// and potentially configured with specific orientations (right, left, front, back).
pros::Distance rightDistance(3);
pros::Distance leftDistance(3);
pros::Distance frontDistance(3);
pros::Distance backDistance(3);

// --- LemLib Definitions ---

// Drivetrain configuration:
// Sets up the physical characteristics of the robot's drivetrain for LemLib.
// - `&left_motors`, `&right_motors`: Pointers to the motor groups for each side.
// - `11.875`: Wheel track width (distance between the centers of the left and right wheels in inches).
// - `lemlib::Omniwheel::NEW_275`: Specifies the type and diameter of the wheels (new 2.75 inch omniwheels).
// - `450`: Max RPM of the motors for the given gearset (600 RPM motors with 1:1.33 gearing = 450 RPM).
// - `2`: External gearing ratio applied to the drivetrain (e.g., 2:1 for 2 times faster).
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.875, lemlib::Omniwheel::NEW_275, 450, 2);

// Odometry Sensors configuration:
// Defines which sensors LemLib will use for calculating the robot's position and heading.
// - `&vertical_tracking_wheel`: Primary vertical tracking wheel.
// - `nullptr`: No secondary vertical tracking wheel.
// - `&horizontal_tracking_wheel`: Primary horizontal tracking wheel.
// - `nullptr`: No secondary horizontal tracking wheel.
// - `&imu`: The Inertial Measurement Unit for precise heading information.
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.25);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0.0625);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// --- PID Definitions for LemLib Chassis ---
// LemLib uses PID (Proportional-Integral-Derivative) controllers for precise movement.
// These settings define how the robot will behave when moving straight or turning.

// Lateral PID (for straight movement and path following):
// kP, kI, kD: PID constants for position control.
// settling error, settling time: How close the robot needs to be to the target and for how long to consider it "settled."
// timeout: Maximum time allowed for the movement.
// exit error, exit time: Conditions to exit movement if not settled but close enough for a certain time.
// min speed: Minimum speed the robot will move at.
lemlib::ControllerSettings lateral_controller(7, 0, 9, 3, 1, 100, 3, 500, 15);

// Angular PID (for turning):
// Similar parameters to lateral PID, but for rotational control.
lemlib::ControllerSettings angular_controller(2, 0, 16, 3, 1, 100, 3, 500, 0);

// Chassis definition:
// Integrates the drivetrain, PID controllers, and odometry sensors into a single LemLib chassis object.
// This object provides methods for movement (e.g., `moveToPose`, `turnTo`).
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// --- Global Variables ---
// 'team': Stores the selected team (e.g., 0 for blue, 1 for red).
// This value would typically be set by the `teamSelector` input.
int team = 0;

// 'autonSelect': Stores the selected autonomous routine index.
// Defaulted to 1 (e.g., for a 'Skills' routine). This value is set by `autonSelector`.
int autonSelect = 1;

/**
 * @brief Runs initialization code.
 * This function will be called in a task on the CPU until the robot has been configured.
 * It's recommended to keep execution time for this mode under a few seconds.
 * This is where you typically set up sensors, motors, and other subsystems.
 */
void initialize() {
    // Set brake modes for drivetrain motors:
    // pros::E_MOTOR_BRAKE_COAST allows the motors to spin freely when not powered.
    // Other options include HOLD (actively holds position) or BRAKE (stops quickly).
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // Reset odometry encoder positions to zero.
    horizontal_encoder.reset_position();
    vertical_encoder.reset_position();

    pros::lcd::initialize(); // Initialize the VEX LCD screen for text output.
    chassis.calibrate();     // Calibrate the LemLib odometry sensors (IMU and encoders).
                             // This is a blocking operation and takes a few seconds.
    teamSelector.calibrate(); // Calibrate the team selector potentiometer (if applicable).
    autonSelector.calibrate(); // Calibrate the autonomous selector potentiometer.

    // Create a PROS task to continuously print robot pose (X, Y, Theta) to the brain screen.
    // This runs in parallel with other robot code.
    pros::Task screen_task([&]() { // The lambda `[&]` captures variables by reference from the surrounding scope.
        while (true) {
            // Print robot's current X coordinate to line 0 of the screen.
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
            // Print robot's current Y coordinate to line 1 of the screen.
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
            // Print robot's current heading (angle in degrees) to line 2 of the screen.
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);

            pros::delay(25); // Small delay to yield CPU, preventing the task from hogging resources.
        }
    });

    // You would typically add code here to read the `autonSelector` potentiometer
    // and set the `autonSelect` variable based on its value. This ensures `autonSelect`
    // is ready before `autonomous()` starts. This logic is also present in `competition_initialize`.
}

/**
 * @brief Runs while the robot is in the disabled state.
 * This function will be started in its own task whenever the robot is disabled
 * via the Field Management System or the VEX Competition Switch.
 * This is a good place to reset global variables or ensure motors are stopped,
 * as motors should not be moving when disabled.
 */
void disabled() {}

/**
 * @brief Runs after initialize(), and before autonomous() or opcontrol().
 * This function is intended for competition-specific initialization routines,
 * such as an autonomous selector on the LCD. This task will exit when the robot
 * is enabled and autonomous or opcontrol starts.
 */
void competition_initialize() {
    // The following code block provides an example of how to implement an autonomous selector
    // using a team selector switch and a potentiometer, displaying options on the screen.

    // Calculate team type (RED/BLUE) based on the teamSelector potentiometer's angle.
    // Assumes `teamSelector.get_angle()` provides a value suitable for this range check.
    // If teamSelector is a digital input, `get_value()` should be used instead of `get_angle()`.
    std::string teamtype = (teamSelector.get_angle() >= 0 && teamSelector.get_angle() <= 165) ? "RED" : "BLUE";

    // Get the current analog value from the autonomous selector potentiometer.
    int potValue = autonSelector.get_value();

    // Calculate the selected autonomous routine number (1-10) using division and clamping.
    // This maps the potentiometer's 0-330 range to 10 integer selections.
    autonSelect = (potValue < 0) ? 1 : (potValue > 329) ? 10 : (potValue / 33) + 1;

    // Map to store autonomous routine descriptions keyed by their selection number.
    // This allows you to display meaningful names on the robot's screen.
    std::map<int, std::string> auton_map = {
        {1, "Auton1"},
        {2, "Auton2"},
        {3, "Auton3"},
        {4, "Auton4"},
        {5, "Auton5"},
        {6, "Auton6"},
        {7, "Auton7"},
        {8, "Auton8"},
        {9, "Auton9"},
        {10, "Auton10"},
    };

    // Loop indefinitely while in pre-autonomous mode to allow selection.
    while (true) {
        // Re-read potentiometer values and re-calculate selections within the loop
        // to allow the user to change selections dynamically before autonomous starts.
        potValue = autonSelector.get_value();
        teamtype = (teamSelector.get_angle() >= 0 && teamSelector.get_angle() <= 165) ? "RED" : "BLUE";
        autonSelect = (potValue < 0) ? 1 : (potValue > 329) ? 10 : (potValue / 33) + 1;

        // Display the selected autonomous routine name on screen line 5.
        // `.c_str()` converts the `std::string` to a C-style string required by `pros::screen::print`.
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, auton_map[autonSelect].c_str());
        // Display the selected team type (RED/BLUE) on screen line 6.
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, teamtype.c_str());

        pros::delay(150); // Delay to prevent rapid screen updates and allow readability.
        pros::screen::erase(); // Clear the screen before the next update for a "refreshing" effect.
    }
}

// --- Namespace for Autonomous Paths ---
// Using a namespace helps organize your autonomous functions and prevents naming conflicts.
namespace autons {
    /**
     * @brief Autonomous routine for Skills.
     * This routine sets the robot's pose and then moves to a specific pose.
     */
    void auton1() { // Placeholder for the Skills Autonomous routine.
        pros::delay(100);       // Small initial delay.
        chassis.setPose(0, 0, 0); // Set initial robot position to (0,0) and heading 0 degrees.

        // Move to a target pose (X: 45, Y: 24, Heading: 90 degrees).
        // `4500` is the timeout in milliseconds.
        // `{ .lead = 0.7, .minSpeed = 60 }`: This is a C++11 brace-enclosed initializer list
        // for `MoveToPointParams`.
        // --- IMPORTANT NOTE ON COMPILER ERROR ---
        // Your screenshot shows "error: cannot convert '<brace-enclosed initializer list>'
        // to 'lemlib::MoveToPointParams'". This error, along with "requires compiler and
        // library support for the ISO C++ 2011 standard," indicates that your compiler/project
        // might not be configured to compile with C++11 (or later) features.
        // You might need to adjust your `project.pros` or `Makefile` to include `-std=c++11`
        // or `-std=gnu++11` compiler options as suggested in the error message.
        // Alternatively, if C++11 is truly not supported, you might need to
        // create a `MoveToPointParams` object explicitly and then pass it, like:
        // lemlib::MoveToPointParams params = {.lead = 0.7, .minSpeed = 60};
        // chassis.moveToPose(45, 24, 90, 4500, params);
        chassis.moveToPose(45, 24, 90, 4500, {.lead = 0.7, .minSpeed = 60});
    }

    // You would define other autonomous routines here (e.g., auton2(), auton3(), etc.)
    // void auton2() { /* ... code for autonomous routine 2 ... */ }

} // namespace autons

/**
 * @brief Runs the user autonomous code.
 * This function will be started in its own task whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the autonomous mode.
 * The selected autonomous routine (from `autonSelect`) will be called here.
 */
void autonomous() {
    // Use an if/else if structure (or a switch statement) to call the specific
    // autonomous routine based on the value set by the autonomous selector.
    if (autonSelect == 1) { // If 'autonSelect' is 1, run the Skills routine.
        autons::auton1();
    }
    // Add 'else if' conditions here for other autonomous routines (e.g., Auton 2, Auton 3, etc.)
    // else if (autonSelect == 2) {
    //     autons::auton2();
    // }
    // else if (autonSelect == 3) {
    //     autons::auton3();
    // }
    // ... up to autonSelect == 10
}

/**
 * @brief Runs the operator control code (driver control).
 * This function will be started in its own task whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the operator control mode.
 * This is where you implement joystick controls for driving and other robot functions.
 */
void opcontrol() {
    // Loop indefinitely to continuously read controller inputs and control the robot.
    while (true) {
        // --- Driving Control (Arcade Style) ---
        // Get the Y-axis value from the left joystick (for forward/backward movement).
        // Values typically range from -127 to 127.
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // Get the X-axis value from the right joystick (for turning).
        // Values typically range from -127 to 127.
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Control the chassis using arcade drive.
        // `leftY` determines the overall speed and direction, `rightX` determines the turning speed.
        chassis.arcade(leftY, rightX);

        // --- Delay for resources ---
        // A small delay to yield control to other PROS tasks (like the screen update task)
        // and reduce CPU usage, preventing the opcontrol loop from consuming all resources.
        pros::delay(25);
    }
}