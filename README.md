# 2602K KryptoKnights VEX V5 Robot Code

This repository contains the PROS C++ code for the 2602K KryptoKnights VEX V5 competitive robotics team. It features a modular structure designed for clarity, maintainability, and easy development of autonomous routines and driver control functionalities.

## Table of Contents
1.  [Project Description](#project-description)
2.  [Key Features](#key-features)
3.  [Code Structure](#code-structure)
4.  [Usage](#usage)
    * [Driver Control (OpControl)](#driver-control-opcontrol)
    * [Autonomous Mode](#autonomous-mode)
5.  [Changelog / Version History](#changelog--version-history)

---

## 1. Project Description

This code operates a VEX V5 competition robot, leveraging the [LemLib](https://lemlib.github.io/lemlib/index.html) motion profiling library for advanced odometry and precise autonomous movements. The project is structured to separate configuration, autonomous logic, and main robot control, making it easier for new team members to understand and contribute.

## 2. Key Features

* **LemLib Integration:** Utilizes LemLib for accurate odometry, advanced motion profiling (e.g., `moveToPose`), and robust drivetrain control.
* **Modular Design:** Code is organized into dedicated files (`robot_config.hpp`, `autons.hpp`/`autons.cpp`, `main.cpp`) for improved readability and maintainability.
* **Centralized Configuration:** All hardware port definitions, drivetrain constants, and PID tunings are consolidated in `robot_config.hpp`.
* **Autonomous Routine Management:** Dedicated `autons.hpp` and `autons.cpp` files for declaring and implementing multiple autonomous strategies.
* **In-Match Autonomous Selection:** Features an interactive autonomous selector using a potentiometer on the VEX V5 Brain's screen.
* **Arcade Driver Control:** Intuitive joystick control for robot movement.
* **Real-time Telemetry:** Displays robot's current X, Y, and Theta (heading) coordinates on the V5 Brain's screen during operation.

## 3. Code Structure

The project is organized into the following key files:

* `src/main.cpp`:
    * The primary entry point for the PROS project.
    * Contains the `pros::Controller` definition, all motor and sensor object instantiations, and the LemLib `chassis` object.
    * Implements the core PROS lifecycle functions: `initialize()`, `disabled()`, `competition_initialize()`, `autonomous()`, and `opcontrol()`.
    * Handles autonomous routine selection and basic driver control logic.
* `include/robot_config.hpp`:
    * **Configuration Header:** Defines all compile-time constants for your robot's hardware.
    * Includes motor port numbers, sensor port numbers (IMU, encoders, potentiometers, distance sensors), drivetrain dimensions, and LemLib PID tuning constants.
    * Designed for easy modification of robot specifications without touching core logic.
* `include/autons.hpp`:
    * **Autonomous Declarations:** Declares the prototype functions for all your specific autonomous routines (e.g., `autons::auton1()`).
    * Part of the `autons` namespace for clear organization.
* `src/autons.cpp`:
    * **Autonomous Implementations:** Contains the actual C++ code for each autonomous routine declared in `autons.hpp`.
    * Each function (`autons::auton1()`, etc.) will outline a specific set of movements and actions for the robot during the autonomous period.

## 4. Usage

### Driver Control (OpControl)

To operate the robot in driver control mode:

1.  Ensure the robot is enabled (via competition switch or Field Management System).
2.  The `opcontrol()` function in `main.cpp` will continuously read joystick inputs.
3.  **Left Joystick Y-axis:** Controls forward and backward movement.
4.  **Right Joystick X-axis:** Controls turning.
5.  (Further subsystem controls, e.g., for intake, arm, mogo, can be added in `opcontrol()` using controller buttons).

### Autonomous Mode

To run an autonomous routine:

1.  During the pre-autonomous period, the V5 Brain's screen will display an **autonomous selector**.
2.  Use the **autonomous selector potentiometer** (defined in `robot_config.hpp`) to scroll through the available autonomous routines (e.g., "Auton1", "Auton2", etc.).
3.  The screen will also display the **selected team** (RED/BLUE) based on another potentiometer or digital switch.
4.  Once the desired autonomous routine is selected, enable the robot in autonomous mode. The `autonomous()` function in `main.cpp` will execute the chosen routine from the `autons` namespace.

## 5. Changelog / Version History

# Changelog: 2602K KryptoKnights VEX V5 Robot Code

This document provides a chronological record of significant changes, refactorings, and bug fixes applied to the 2602K VEX V5 Robot Code.

## Version 1.0.0 - Initial Codebase (Pre-July 2025 Refactor)

* **Initial State:** Monolithic `main.cpp` containing all drivetrain, sensor, LemLib, autonomous, and driver control logic.
* **Features:**
    * LemLib-based odometry and chassis control.
    * Basic autonomous selection and execution.
    * Arcade driver control.
    * On-screen telemetry for robot pose.
    * Motors, IMU, rotation sensors, potentiometers, and distance sensors defined directly in `main.cpp`.
* **Known Issues:** Large `main.cpp` file, making it less modular and harder to maintain or expand.

## Version 1.1.0 - Modular Refactoring Attempt (July 4, 2025)

* **Changes:**
    * Attempted to introduce a `Robot` class (`robot.hpp`, `robot.cpp`) to encapsulate hardware and LemLib objects for better organization.
    * Planned to move autonomous functions into `autons.hpp` and `autons.cpp`.
    * Introduced `robot_config.hpp` for centralized hardware and constant definitions.
* **Issues Introduced:**
    * **"Use of undeclared identifier 'robot'"**: This error occurred because `main.cpp` attempted to use a global `robot` object that was declared in `robot.hpp` but was not correctly linked or defined due to the incomplete refactoring or misunderstanding of the intended scope.

## Version 1.2.0 - Refactored for Flat Structure & Bug Fixes (July 4-5, 2025)

* **Changes:**
    * **Reverted `main.cpp` to a flatter structure:** Removed the `Robot` class concept and re-integrated direct definitions of motors, sensors, and LemLib objects into `main.cpp`.
    * **Fully Integrated `robot_config.hpp`:** All hardware ports, drivetrain constants, and PID tunings were successfully moved to and referenced from `robot_config.hpp`, centralizing configuration.
    * **Fully Integrated `autons.hpp` and `autons.cpp`:** All autonomous routine declarations and implementations were successfully moved into these dedicated files, cleaning up `main.cpp`.
    * **Fixed C++ Standard Resetting:** Resolved the issue where the `project.pros` file would revert the C++ standard flag (`-std=gnu++20`) by ensuring correct direct editing of `project.pros` and removing conflicting entries.
    * **Removed Erroneous `src/robot_config.cpp`:** Identified and deleted the incorrectly created `src/robot_config.cpp` file, which was an unnecessary compilation unit for a header-only configuration.
* **Outcome:**
    * Code is now modularized into `main.cpp`, `robot_config.hpp`, `autons.hpp`, and `autons.cpp`.
    * Compilation errors resolved.
    * C++ standard (`gnu++20`) now persists correctly.
* **Current Status:** Stable and organized codebase for continued development.

## Version 1.2.1 - Cache Management Notes (July 5, 2025)

* **Change:** Clarified that the `.cache` folder can be safely deleted to free up disk space or resolve minor build/IDE issues, as it only contains temporary build artifacts and indexing data.
* **Outcome:** Improved understanding of project file management.

## Version 1.3.0 - Cross-File Access & UI Optimization (July 5, 2025)

* **Features/Improvements:**
    * **Enabled `chassis` access in `autons.cpp`:** Added `extern lemlib::Chassis chassis;` and `#include "lemlib/api.hpp"` to `include/main.h` to allow `autons.cpp` to properly access the globally defined `chassis` object.
    * **Enabled `selectedAuton` access in `autons.cpp`:** Added `extern int selectedAuton;` to `include/main.h` to allow `autons.cpp` to use the global autonomous selection variable.
    * **Optimized `competition_initialize()` function:**
        * Moved `std::map` initialization outside the `while` loop to prevent redundant re-creation on every iteration, significantly improving efficiency.
        * Refined variable declarations (`potValue`, `selectedAuton`, `teamtype`) to occur only once inside the `while` loop, avoiding unnecessary re-declarations.
    * **Improved Autonomous Selection Logic:** Switched the `autonomous()` function's `if-else if` chain to a more readable and scalable `switch` statement for better autonomous routine dispatch.
    * **Added Subsystem Files:** Introduced `subsystems.hpp` and `subsystems.cpp` to further modularize robot functionality, allowing for organized management of individual robot mechanisms (e.g., intake, lift, arm).
* **Outcome:** Enhanced code modularity, improved runtime performance of the pre-autonomous selector, and cleaner autonomous routine dispatch, with a clear structure for future subsystem development.

---