# VEX V5 PROS Robot Code - 2602K RobotCode

This project provides a clean and commented starting point for a VEX V5 robot using the PROS and LemLib libraries. It sets up a basic drivetrain, integrates advanced odometry for precise movement, and includes a foundation for running autonomous routines and basic driver control.

## What This Code Does

At its core, this code makes your robot drive smoothly and accurately using LemLib. It allows you to select different autonomous routines before a match and gives you manual control over the drivetrain during the driver control period. While currently focused on driving, it's designed to be easily expandable for more robot mechanisms.

## How the Robot is Set Up (Hardware & Configuration)

The `main.cpp` file begins by including necessary libraries like `main.h` (the main PROS header) and `lemlib/api.hpp` for the LemLib functions.

It then declares all the **robot's hardware**:
* A `pros::Controller` is set up for driver input.
* The **drivetrain motors** are defined, indicating which ports they're on and their gearing (blue for 600 RPM motors). Negative port numbers usually mean the motor's direction is reversed.
* For **accurate odometry (tracking the robot's position)**, an `Imu` (Inertial Measurement Unit) is used to sense heading, and two `pros::Rotation` sensors act as tracking wheels to measure linear movement.
* A `pros::adi::Potentiometer` and a `pros::adi::DigitalIn` are included to allow for selecting autonomous routines and team side before a match.
* Placeholder `pros::Distance` sensors are defined, ready to be used if your robot has them.

Following hardware definitions, **LemLib is configured**:
* A `lemlib::Drivetrain` object tells LemLib about your robot's wheels, their size, and how fast the motors can spin.
* `lemlib::OdomSensors` brings together your IMU and tracking wheels, allowing LemLib to constantly calculate your robot's precise X, Y, and heading.
* `lemlib::ControllerSettings` objects define **PID (Proportional-Integral-Derivative) constants**. These are crucial for LemLib to control your robot's movement accurately, both for driving straight (`lateral_controller`) and turning (`angular_controller`). They help the robot reach its target smoothly without overshooting.
* Finally, a `lemlib::Chassis` object is created, combining all these drivetrain, sensor, and PID settings into one powerful object that handles all your robot's driving and odometry needs.

Some simple **global variables** like `team` and `autonSelect` are set up to keep track of match settings, like which autonomous routine is chosen.

## How the Code Runs (Program Flow)

VEX PROS organizes robot code into specific functions that run at different times:

### `initialize()`
This function runs once when the robot starts up. Here, the code:
* Sets your drive motors to `COAST` mode.
* Resets the odometry tracking wheel encoders.
* Initializes the VEX Brain's LCD screen.
* **Calibrates the LemLib chassis**, which is very important for accurate odometry (especially for the IMU).
* It also starts a background task to constantly **display your robot's X, Y, and heading on the brain screen**, which is super helpful for debugging odometry.

### `disabled()`
This short function runs continuously whenever the robot is disabled (e.g., between matches). It's a good spot for any cleanup or to ensure motors are stopped.

### `competition_initialize()`
This function runs after `initialize()` and before `autonomous()` or `opcontrol()`. It's typically used for things like:
* **Autonomous selection**: The code includes a *commented-out example* of a sophisticated autonomous selector. You can uncomment and adapt this section to use the potentiometer and digital input to choose different autonomous routines and display them on the brain screen. This setup is often preferred over putting selection logic in `initialize()` so that it can be easily updated after the robot has finished its initial boot-up.

### `autonomous()`
This is where your robot performs its pre-programmed actions without driver input.
* It checks the `autonSelect` variable (which would be set in `competition_initialize` or `initialize`) and calls the corresponding autonomous routine from the `autons` namespace.
* Currently, if `autonSelect` is 1, it runs `autons::auton1()`.

### `opcontrol()`
This is the driver control period. The code continuously runs a loop that:
* Reads joystick inputs from the `pros::Controller`.
* Uses `chassis.arcade()` to control the drivetrain. This means the left joystick's up/down movement controls forward/backward, and the right joystick's left/right movement controls turning.
* Includes a small `pros::delay(25)` to prevent the code from hogging the CPU, allowing other PROS tasks to run smoothly.
* **Note**: This version of the driver control code is simplified and does not contain logic for any additional mechanisms like arms, intakes, or mobile goal systems. You would add that functionality here as your robot design progresses.

## Autonomous Routines (`namespace autons`)

The `autons` namespace is where all your specific autonomous paths are defined. This keeps your autonomous code organized.

* **`autons::auton1()`**:
    * This is set up as a "Skills" routine example.
    * It first sets the robot's starting position (`chassis.setPose(0, 0, 0)`).
    * Then, it uses `chassis.moveToPose()` to command the robot to drive to a specific X, Y coordinate and end at a particular heading, with parameters to fine-tune its motion.

## Getting Started & Expanding

1.  **Configure Ports**: Make sure all motor and sensor ports in `main.cpp` accurately reflect your robot's wiring.
2.  **Tune PIDs**: For precise movement, you'll need to tune the `lateral_controller` and `angular_controller` PID values in `main.cpp` based on your robot's weight, friction, and motor power. LemLib's documentation provides guides for this.
3.  **Enable Autonomous Selector**: If you want to use the included selector, uncomment and customize the `competition_initialize()` section.
4.  **Add Subsystem Control**: Implement code in `opcontrol()` for any additional robot mechanisms (e.g., an arm, claw, or intake).
5.  **Create More Autons**: Duplicate and modify `autons::auton1()` to create additional autonomous routines as needed for your competition strategy.

This code provides a strong, well-structured foundation for your VEX robot, ready for further development!