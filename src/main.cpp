#include "main.h" // PROS main header
#include "lemlib/api.hpp" // LemLib API for odometry and chassis control
#include "pros/adi.hpp"
#include <map>
#include <string>

// --- Controller Definition ---
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Motor Definitions ---
// Right Motors: Ports 11, 13, 12. Negative signs on ports usually indicate reversed motors.
pros::MotorGroup right_motors({11, 13, 12}, pros::MotorGearset::blue);
// Left Motors: Ports 14, 15, 16. Negative signs on ports usually indicate reversed motors.
pros::MotorGroup left_motors({-14, -15, -16}, pros::MotorGearset::blue);

// --- Sensor Definitions ---
// Inertial Sensor (IMU) on port 2 for heading and rotation
pros::Imu imu(2);

// Horizontal Odometry Tracker using a Rotation Sensor on port -3 (reversed)
pros::Rotation horizontal_encoder(-3);
// Vertical Odometry Tracker using a Rotation Sensor on port 17
pros::Rotation vertical_encoder(17);

// Autonomous Selector Potentiometer on ADI port 6
pros::adi::Potentiometer autonSelector(6);
// Team Selector Digital Input (e.g., bumper switch) on ADI port 7
pros::adi::Potentiometer teamSelector(7);

// Distance Sensors (for odometry resets)
pros::Distance rightDistance(3);
pros::Distance leftDistance(3);
pros::Distance frontDistance(3);
pros::Distance backDistance(3);

// --- LemLib Definitions ---
// Drivetrain configuration:
// - Left and right motor groups
// - Wheel track width (distance between left and right wheels in inches)
// - Wheel diameter (Omniwheel::NEW_275 is a pre-defined 2.75 inch wheel)
// - RPM (Max motor RPM for the given gearset, 450 for blue)
// - Gearing (External gearing, if any, applied to the drivetrain)
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.875, lemlib::Omniwheel::NEW_275, 450, 2);

// Odometry Sensors configuration:
// - Horizontal tracking wheel: &horizontal_encoder, wheel diameter, offset from robot center
// - Vertical tracking wheel: &vertical_encoder, wheel diameter, offset from robot center
// - IMU: &imu for heading
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.25);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0.0625);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// --- PID Definitions for LemLib Chassis ---
// Lateral PID (for straight movement and path following):
// kP, kI, kD, settling error, settling time, timeout, exit error, exit time, min speed (new params for newer LemLib)
lemlib::ControllerSettings lateral_controller(7, 0, 9, 3, 1, 100, 3, 500, 15);
// Angular PID (for turning):
// kP, kI, kD, settling error, settling time, timeout, exit error, exit time, min speed
lemlib::ControllerSettings angular_controller(2, 0, 16, 3, 1, 100, 3, 500, 0);

// Chassis definition using drivetrain, PIDs, and odometry sensors
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// --- Global Variables ---
int team = 0; // 0 for blue, 1 for red (based on teamSelector potentially)
int autonSelect = 1; // Default autonomous routine selection (e.g., 1 for Skills)

/**
 * @brief Runs initialization code.
 * This function will be called in a task on the CPU until the robot has been configured.
 * It's recommended to keep execution time for this mode under a few seconds.
 * This is where you typically set up sensors, motors, and other subsystems.
 */
void initialize() {
    // Set brake modes for drivetrain motors
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // Reset odometry encoder positions
    horizontal_encoder.reset_position();
    vertical_encoder.reset_position();

    pros::lcd::initialize(); // Initialize the VEX LCD (for basic prints)
    chassis.calibrate();     // Calibrate the LemLib odometry sensors (IMU, encoders)
	teamSelector.calibrate(); // Calibrate team selector
	autonSelector.calibrate(); // Calibrate autonomous selector
    
    // Create a task to continuously print robot pose (X, Y, Theta) to the brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // Print robot location to the brain screen
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);      // X coordinate
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);      // Y coordinate
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta); // Heading (angle)

            pros::delay(25); // Small delay to save resources and prevent blocking
        }
    });

    // You would typically add code here to read the autonSelector potentiometer
    // and set the 'autonSelect' variable based on its value before competition_initialize.
    // For example, a while loop similar to what was in competition_initialize previously.
}

/**
 * @brief Runs while the robot is in the disabled state.
 *
 * This function will be started in its own task whenever the robot is disabled
 * via the Field Management System or the VEX Competition Switch.
 * This is a good place to reset global variables or ensure motors are stopped.
 */
void disabled() {}

/**
 * @brief Runs after initialize(), and before autonomous() or opcontrol().
 *
 * This function is intended for competition-specific initialization routines,
 * such as an autonomous selector on the LCD.
 * This task will exit when the robot is enabled and autonomous or opcontrol starts.
 */
void competition_initialize() {
    // The following code block was commented out in the original submission.
    // It provides an example of how to implement an autonomous selector using
    // a team selector switch and a potentiometer, displaying options on the screen.
    std::string teamtype = (teamSelector.get_angle() >= 0 && teamSelector.get_angle() <= 165) ? "RED" : "BLUE"; // Initialize team type string
	autonSelect = std::clamp(autonSelector.get_value() / 33, 1, 10); // Initialize autonomous value
    // Map to store autonomous routine descriptions keyed by their selection number
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

    while (true) {
        // Toggle team (RED/BLUE) on new press of the teamSelector digital input
        teamtype = (teamSelector.get_angle() >= 0 && teamSelector.get_angle() <= 165) ? "RED" : "BLUE";
		autonSelect = std::clamp(autonSelector.get_value() / 33, 1, 10);
        // Display selected autonomous routine description on the screen
        // autonSelect should be set by reading the autonSelector potentiometer here,
        // typically in a range-based 'if/else if' structure.
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, auton_map[autonSelect].c_str());
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, teamtype.c_str()); // Display selected team type

        pros::delay(150); // Delay to prevent rapid updates and allow screen to be read
        pros::screen::erase(); // Clear the screen for the next update
    }
}

// --- Namespace for Autonomous Paths ---
namespace autons {
    /**
     * @brief Autonomous routine for Skills.
     * This routine sets the robot's pose and then moves to a specific pose.
     */
    void auton1() { // Skills Auton
        pros::delay(100);       // Small initial delay
        chassis.setPose(0, 0, 0); // Set initial robot position to (0,0) and heading 0
        // Move to a target pose (X: 45, Y: 24, Heading: 90 degrees)
        // With a timeout of 4500ms and custom movement parameters (lead, minSpeed)
        chassis.moveToPose(45, 24, 90, 4500, {.lead = 0.7, .minSpeed = 60});
    }
} // namespace autons

/**
 * @brief Runs the user autonomous code.
 *
 * This function will be started in its own task with the default priority and stack size
 * whenever the robot is enabled via the Field Management System or the VEX Competition Switch
 * in the autonomous mode. Alternatively, this function may be called in initialize() or opcontrol()
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task will be stopped.
 * Re-enabling the robot will restart the task, not resume it from where it left off.
 */
void autonomous() {
    // Select and run the chosen autonomous routine based on 'autonSelect' variable.
    // (0 = blue side auton, 1 = red side auton, or specific routine index)
    if (autonSelect == 1) { // If 'autonSelect' is 1, run the Skills routine
        autons::auton1();
    }
    // Add 'else if' conditions here for other autonomous routines
    // e.g., else if (autonSelect == 2) { autons::auton2(); }
}

/**
 * @brief Runs the operator control code.
 *
 * This function will be started in its own task with the default priority and stack size
 * whenever the robot is enabled via the Field Management System or the VEX Competition Switch
 * in the operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the operator control task
 * will be stopped. Re-enabling the robot will restart the task, not resume it
 * from where it left off.
 */
void opcontrol() {
    while (true) {
        // --- Driving Control (Arcade Style) ---
        // Get joystick values for left Y-axis (forward/backward) and right X-axis (turning)
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Control the chassis using arcade drive
        // 'leftY' controls forward/backward, 'rightX' controls turning
        chassis.arcade(leftY, rightX);

        // --- Delay for resources ---
        // A small delay to yield control to other PROS tasks and reduce CPU usage.
        pros::delay(25);
    }
}