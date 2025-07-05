#include "main.h" // PROS main header
#include "lemlib/api.hpp" // LemLib API for odometry and chassis control
#include "robot_config.hpp"
#include "autons.hpp"
#include <map>
#include <string>

// --- Controller Definition ---
// Initializes the primary VEX V5 controller connected to the robot.
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Motor Definitions ---
// MotorGroup is a LemLib class that allows you to control multiple motors as a single unit.
// Using constants from robot_config.hpp for port numbers.
pros::MotorGroup right_motors({PORT_RIGHT_MOTOR_1, PORT_RIGHT_MOTOR_2, PORT_RIGHT_MOTOR_3}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({PORT_LEFT_MOTOR_1, PORT_LEFT_MOTOR_2, PORT_LEFT_MOTOR_3}, pros::MotorGearset::blue);

// --- Sensor Definitions ---
// Using constants from robot_config.hpp for port numbers.
pros::Imu imu(PORT_IMU);
pros::Rotation horizontal_encoder(PORT_HORIZONTAL_ENCODER);
pros::Rotation vertical_encoder(PORT_VERTICAL_ENCODER);
pros::adi::Potentiometer autonSelector(PORT_AUTON_SELECTOR_POT);
pros::adi::Potentiometer teamSelector(PORT_TEAM_SELECTOR_POT);
pros::Distance rightDistance(PORT_DISTANCE_RIGHT);
pros::Distance leftDistance(PORT_DISTANCE_LEFT);
pros::Distance frontDistance(PORT_DISTANCE_FRONT);
pros::Distance backDistance(PORT_DISTANCE_BACK);

// --- LemLib Definitions ---
// Drivetrain configuration, using constants from robot_config.hpp
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, TRACK_WIDTH, lemlib::Omniwheel::NEW_275, MOTOR_RPM, DRIVETRAIN_GEARING);

// Odometry Tracking Wheel configurations, using constants from robot_config.hpp
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, HORIZONTAL_TRACKING_OFFSET);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, VERTICAL_TRACKING_OFFSET);

// Odometry Sensors configuration
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// PID Controller Settings, using constants from robot_config.hpp
lemlib::ControllerSettings lateral_controller(LATERAL_KP, LATERAL_KI, LATERAL_KD, LATERAL_SETTLE_ERR, LATERAL_SETTLE_TIME, LATERAL_TIMEOUT, LATERAL_EXIT_ERR, LATERAL_EXIT_TIME, LATERAL_MIN_SPEED);
lemlib::ControllerSettings angular_controller(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, ANGULAR_SETTLE_ERR, ANGULAR_SETTLE_TIME, ANGULAR_TIMEOUT, ANGULAR_EXIT_ERR, ANGULAR_EXIT_TIME, ANGULAR_MIN_SPEED);

// Chassis definition: Integrates all LemLib components
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

    std::string teamtype = (teamSelector.get_angle() >= 0 && teamSelector.get_angle() <= 165) ? "RED" : "BLUE"; // Initialize team type string
	int potValue = autonSelector.get_value();
	int selectedAuton = (potValue < 0) ? 1 : (potValue > 329) ? 10 : (potValue / 33) + 1;
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
        // Update the Autonomous Selector and Team Selector
        int potValue = autonSelector.get_value();
        std::string teamtype = (teamSelector.get_angle() >= 0 && teamSelector.get_angle() <= 165) ? "RED" : "BLUE"; // Initialize team type string
	    int selectedAuton = (potValue < 0) ? 1 : (potValue > 329) ? 10 : (potValue / 33) + 1;
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
        auton1();
    } else if (autonSelect == 2) {
        auton2();
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
