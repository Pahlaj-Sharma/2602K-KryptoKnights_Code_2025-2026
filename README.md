# 2602K-RobotCode

## 🤖 Project Overview

This repository contains the PROS C++ codebase for VEX V5 Robotics Team 2602K. Our robot is engineered to compete in the Push Back game. Our primary strategy focuses on creating a robust autonomous program.

This project leverages the **PROS** (Purdue Robotics Operating System) framework for all robot low-level control, sensor integration, and motor management. For advanced autonomous movement and precise robot localization, we integrate the **LemLib** library, utilizing its robust odometry and motion control capabilities.

## ✨ Key Features

This software enables the following functionalities for our robot:

* **Sophisticated Autonomous Routines:**
    * Includes 10 pre-programmed autonomous paths designed for various starting positions and strategic objectives on the field.
    * Utilizes LemLib for precise odometry-driven motion control, including path following (`chassis.moveToPose()`) and accurate turns (`chassis.turnTo()`).
    * Features a custom `moveLinear()` function to precisely drive the robot a specified distance forward or backward based on its current heading.
    * Autonomous routine selection and team color (Red/Blue) are dynamically chosen using physical potentiometers before the match starts.
    * List the names or descriptions of your specific autonomous routines here (corresponding to Auton1-Auton10). For example:
        * Auton1: Red Side - Offensive High Goal
        * Auton2: Blue Side - Defensive Block with Low Goal Score
        * Auton3: Skills Challenge - Autonomous Only Routine
* **Intuitive Driver Control Interface:**
    * **Drive Control Scheme:** Arcade drive, where the left joystick Y-axis controls forward/backward movement and the right joystick X-axis controls turning. The drivetrain consists of 6 motors for powerful and precise movement.
    * **Manipulator Controls:** [CLEARLY DESCRIBE HOW YOUR ROBOT'S MANIPULATORS (arm, intake, claw, etc.) are controlled. Example: "R1 button activates the intake to spin inwards, R2 button spins the intake outwards." OR "L1 button raises the arm, L2 button lowers the arm." Provide details for all manipulators.]
* **Dynamic PID Control Tuning:**
    * A custom `chassisPID()` function allows for on-the-fly switching between predefined PID tuning presets ("normal", "fast", "precise") or setting custom PID values for LemLib's lateral and angular controllers, enabling rapid adaptation to different autonomous segments.
* **Modular and Maintainable Codebase:**
    * The code is structured into distinct, logical components representing robot subsystems (e.g., Drivetrain, Intake, Arm) defined in `subsystems.hpp` and `robot_config.hpp`. This organization enhances readability, simplifies debugging, and supports collaborative development.
    * Custom C++ classes and structs are utilized for managing individual robot subsystems and complex functionalities.
* **Comprehensive Sensor Integration & Localization:**
    * The robot incorporates various VEX V5 sensors to perceive its environment and execute precise maneuvers.
    * **Odometry System:** Utilizes LemLib's odometry system, fusing data from an Inertial Measurement Unit (IMU) and two VEX Rotation Sensors acting as horizontal and vertical tracking wheels.
    * **Localization Correction (`resetOdometry()`):** A custom `resetOdometry()` function uses readings from four VEX V5 Distance Sensors (Front, Back, Left, Right) to periodically correct the robot's estimated position (X and Y coordinates) against known field boundaries or walls, enhancing long-term localization accuracy.
    * Continuous robot pose (X, Y, Theta) is printed to the V5 Brain screen, and vital robot information (battery, motor temperature, selected auton/team) is displayed on the V5 Controller screen for real-time monitoring.

## 💻 Software & Development Tools

* **Programming Language:** C++ (PROS API)
* **Robot Operating System:** PROS (Purdue Robotics Operating System)
* **Integrated Development Environment (IDE):** Visual Studio Code with the official PROS Extension.
* **Core Libraries:**
    * **PROS API:** The standard library for interacting with VEX V5 hardware components.
    * **LemLib:** An advanced motion control library providing robust odometry, PID controllers, and path planning capabilities.
    * **Custom Subsystem Libraries:** User-defined headers (`robot_config.hpp`, `autons.hpp`, `subsystems.hpp`) for organized code structure and custom robot functionalities.
* **Supporting Tools:**
    * **PROS CLI:** Used for project creation, firmware updates, and direct robot interaction.
    * [LIST ANY EXTERNAL TOOLS USED FOR DEVELOPMENT. Example: "A custom Python script for generating autonomous paths for LemLib from CSV files."]

## 🎮 Robot Operation Guide

### Autonomous Mode Execution

* **Autonomous Routine Selection:** On the V5 Robot Brain screen, the current autonomous routine and team type (RED/BLUE) are displayed. Use the dedicated autonomous selector potentiometer (`autonSelector`) to choose from the 10 available autonomous routines. Use the team selector potentiometer (`teamSelector`) to set the robot's alliance color. The selected routine and team will also be shown on the V5 Controller screen.
* **Starting Autonomous:** After confirming your selections, place the robot precisely in its designated starting position on the competition field for the chosen autonomous routine. The autonomous program will initiate automatically when the robot is enabled by the field control system.

### Driver Control (OpControl)

* **Enabling Driver Control:** Driver control mode will activate automatically after the autonomous period concludes, or immediately if no autonomous routine is selected (or autonomous time runs out) and the robot is enabled by the field control system.
* **Controller Mappings:**
    * **Drivetrain (Arcade Control):**
        * Left Joystick Y-axis: Controls forward and backward movement.
        * Right Joystick X-axis: Controls turning (rotation) of the robot.
    * **Manipulators:** [PROVIDE THE EXACT BUTTON/JOYSTICK MAPPINGS FOR EACH OF YOUR ROBOT'S MANIPULATORS. Example: "R1 Button: Activates the intake rollers inwards (collecting)." "R2 Button: Activates the intake rollers outwards (expelling)." "L1 Button: Raises the robot's arm." "L2 Button: Lowers the robot's arm." Be thorough for all mechanisms.]

## ⚙️ Configuration & Tuning Notes

Successful robot performance in VEX Robotics heavily relies on precise calibration and diligent tuning. Pay close attention to these critical areas:

* **LemLib Controller Tuning (Lateral & Angular):** The `LATERAL_KP`, `LATERAL_KI`, `LATERAL_KD` and `ANGULAR_KP`, `ANGULAR_KI`, `ANGULAR_KD` constants, along with their `FAST` (`F_`) and `PRECISE` (`P_`) variants, defined in `robot_config.hpp`, are crucial for accurate motion control. These PID gains must be carefully tuned for your specific robot's weight, friction, motor power, and drivetrain characteristics to achieve smooth and precise movements. Refer to the official [LemLib Documentation](https://lemlib.readthedocs.io/en/latest/docs/tuning/tuning.html) for comprehensive tuning guides.
* **Odometry System Calibration:** The accuracy of robot positioning depends on the precise calibration of your odometry sensors and parameters. Ensure that `TRACK_WIDTH`, `WHEEL_RPM`, `HORIZONTAL_TRACKING_OFFSET`, and `VERTICAL_TRACKING_OFFSET` (defined in `robot_config.hpp`) are measured and configured with extreme precision for your robot's physical setup. Regular calibration of the VEX V5 Inertial Sensor (`imu.calibrate()` in `initialize()`) is also essential.
* **Distance Sensor Offsets for `resetOdometry()`:** The `DS_FRONT_CENTER`, `DS_BACK_CENTER`, `DS_LEFT_CENTER`, and `DS_RIGHT_CENTER` constants (expected to be in `robot_config.hpp`) representing the physical distance from your robot's center to each distance sensor are critical for accurate pose correction. These values must be measured precisely. The `threshold` parameter passed to `resetOdometry()` also needs careful tuning based on expected sensor noise and desired correction aggressiveness.
* **Motor Inversion Settings:** Double-check that each `pros::Motor` instance's inversion setting (`true` for inverted, `false` for not inverted) correctly matches the physical orientation of the motor and its desired direction of rotation for forward movement. Incorrect settings will lead to unexpected drivetrain behavior.
* **Potentiometer Calibration:** The `potValue` ranges for `autonSelector` and `teamSelector` in `competition_initialize()` (e.g., the divisions for `selectedAuton` and `teamtype`) must be carefully calibrated to your specific potentiometers' physical ranges to ensure correct routine and team selection across their full rotation.

## 📜 Copyright and License

* **Copyright 2025-2026 Pahlaj Sharma. All rights reserved.**
* This project is currently provided with all rights reserved by the author. No unauthorized reproduction, distribution, or modification is permitted without explicit written consent.

---

**Project Lead:** Pahlaj Sharma
**Team:** 2602K
**Last Updated:** July 27, 2025
**Current Software Version:** 1.81