# VEX Robotics Competition Code

This repository contains the PROS C++ code for our VEX V5 competition robot. It utilizes the LemLib library for advanced autonomous routines and odometry.

## Project Structure

* `src/main.cpp`: The main source file containing all robot logic.

## Software and Libraries

* **PROS (PROS for V5)**: The primary firmware and API used for programming the VEX V5 robot.
* **LemLib**: A motion planning library for VEX robots, used here for autonomous path generation and odometry.
* **VEX Rotation Sensor C++ API**: Used for precise tracking and measurement with the VEX Rotation Sensor.

## Robot Configuration

### Motors

* **Right Drive Motors**: Ports 11, 13, 12 (Blue Gearset)
* **Left Drive Motors**: Ports -14, -15, -16 (Blue Gearset)
* **Front Intake Motor**: Port 9 (Green Gearset)
* **Hooks Intake Motor**: Port -18 (Blue Gearset)
* **Arm Motor**: Port 20 (Green Gearset)

### Sensors

* **Inertial Sensor (IMU)**: Port 2
* **Horizontal Odometry Encoder**: Port -3
* **Vertical Odometry Encoder**: Port 17
* **Arm Rotation Sensor**: Port 6
* **Color Sensor**: Port 2
* **Auton Selector Potentiometer**: Port 6 (ADI)
* **Team Selector Digital Input**: Port 7 (ADI)
* **Distance Sensors**: Right, Left, Front, Back (all on Port 3 - *Note: This might be a wiring/port conflict and should be verified.*)

### Pneumatics/Digital Outputs

* **Mogo Mechanism**: Port 2 (ADI)
* **Left Sweeper**: Port 4 (ADI)
* **Right Sweeper**: Port 3 (ADI)
* **Intake Lift**: Port 5 (ADI)

## Autonomous Mode (LemLib)

The robot utilizes LemLib for its autonomous routines, leveraging odometry for accurate positioning and path following.

### Drivetrain Configuration

* **Left Motors**: `left_motors`
* **Right Motors**: `right_motors`
* **Wheel Track**: 11.875 inches
* **Wheel Type**: LemLib Omniwheel (NEW_275)
* **TPR (Ticks Per Rotation)**: 450
* **Gear Ratio**: 2

### Odometry Sensors

* **Vertical Tracking Wheel**: `vertical_encoder` (LemLib Omniwheel NEW_275, offset 0.0625 inches)
* **Horizontal Tracking Wheel**: `horizontal_encoder` (LemLib Omniwheel NEW_275, offset -5.25 inches)
* **IMU**: `imu`

### PID Controllers

* **Lateral Controller Settings**:
    * kP: 7
    * kI: 0
    * kD: 9
    * Settling Error: 3
    * Settling Time: 100 ms
    * Early Exit Range: 3 inches
    * Timeout: 500 ms
    * Min Speed: 15
* **Angular Controller Settings**:
    * kP: 2
    * kI: 0
    * kD: 16
    * Settling Error: 3
    * Settling Time: 100 ms
    * Early Exit Range: 3 inches
    * Timeout: 500 ms
    * Min Speed: 0

### Autonomous Routines (`autons` namespace)

The `autonomous()` function selects one of several predefined autonomous routines based on the `autonSelect` variable. This variable is intended to be set by the `autonSelector` potentiometer during `competition_initialize()`.

* `auton1()`: Skills autonomous routine.
* `auton2()`: Negative Side AWP (Alliance Win Point) routine (gets alliance stake).
* `auton3()`: Positive Side AWP (gets middle rings).
* `auton4()`: Negative Side Eliminations (gets alliance stake).
* `auton5()`: Positive Side Eliminations (gets middle rings).
* `auton6()`: Solo AWP routine.
* `auton7()`: Positive Side Eliminations (NO MIDDLE rings).
* `auton8()`: Positive Side AWP (NO MIDDLE rings).
* `auton9()`: (Placeholder)
* `auton10()`: (Placeholder)

## Operator Control Mode (`opcontrol`)

The `opcontrol()` function handles the driver control period.

### Controls:

* **Drivetrain**: Arcade control using Left Y (forward/backward) and Right X (turning) on the master controller.
* **Intake Toggle**: `Y` button
* **Arm Up (PID to LBHEIGHT)**: `R1` button
* **Arm Down (PID to 3 degrees)**: `R2` button
* **Arm Manual Up**: Hold `L1` button
* **Arm Manual Down**: Hold `DOWN` arrow button
* **Left Sweeper Toggle**: `L2` button
* **Right Sweeper Toggle**: `UP` arrow button
* **Mogo Clamp Toggle**: `RIGHT` arrow button
* **Outtake (Hooks & Front)**: Hold `LEFT` arrow button
* **Color Sort Toggle**: `B` button

## Subsystems and Helper Functions

* `subsystem::intake()`: Toggles the intake motors on/off.
* `subsystem::armSpin(bool reset)`: Controls the arm movement. If `reset` is false and `armpicked` is false, it moves the arm to `LBHEIGHT` using PID. If `reset` is true, it moves the arm to 3 degrees (reset position). Otherwise, it moves the arm to 140 degrees (outtake position).
* `subsystem::mogo()`: Toggles the mogo mechanism (clamp/release).
* `armSpinTo(float angle, float settle_error, float scale, int timeout)`: PID control function for the arm, moving it to a specified angle.
* `colorSorting()`: A task that continuously checks the color sensor and outtakes rings of the opposite team's color if `colorSort` is enabled and the intake is spinning.

## Initialization (`initialize`)

* Sets drivetrain brake modes to coast.
* Resets horizontal and vertical encoder positions.
* Disables color sensor gestures.
* Initializes the PROS LCD.
* Calibrates the LemLib chassis (important for accurate odometry).
* Starts `colorSorting()` and a screen printing task for odometry data.

## Competition Initialization (`competition_initialize`)

* (Commented out) This section is intended for an autonomous and team selector using the potentiometer and digital input. It displays the selected autonomous routine and team (Red/Blue) on the brain screen.

## Notes and Potential Improvements

* **Distance Sensor Port Conflict**: All `pros::Distance` sensors are currently assigned to Port 3. This is likely an error and will cause conflicts. Each distance sensor needs a unique smart port.
* **Auton Selector Implementation**: The `competition_initialize` function that handles autonomous and team selection is commented out. It needs to be uncommented and potentially refined for proper in-competition use.
* **Arm PID Tuning**: The `armPID` values (kP=3, kI=0, kD=30) might need further tuning for optimal arm performance.
* **Autonomous Path Tuning**: The `moveToPose` and `moveToPoint` parameters (`lead`, `minSpeed`, `maxSpeed`, `earlyExitRange`, `timeout`, `forwards`) in the autonomous routines will likely require extensive tuning for precise robot movement.
* **Error Handling**: Consider adding more robust error handling, especially for sensor readings and motor operations.
* **Code Clarity**: Some sections could benefit from additional comments explaining the logic, especially complex autonomous sequences.
* **Magic Numbers**: Several 'magic numbers' (e.g., motor velocities, delays, specific angles) are present throughout the code. Defining these as constants or enumerations could improve readability and maintainability.
