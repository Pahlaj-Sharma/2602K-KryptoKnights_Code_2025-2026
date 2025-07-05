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

---