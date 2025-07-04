# 2602K KryptoKnights VEX V5 Robot Code (2025-2026 Season)

## 1. Project Overview

This repository contains the PROS (PROS Robot Operating System) C++ code for the 2602K KryptoKnights VEX V5 Competition Robot for the 2025-2026 season. The code leverages the LemLib library for advanced odometry, precise motion profiling, and robust chassis control, aiming for highly consistent and reliable autonomous routines and intuitive driver control.

**Key Features:**
* **LemLib-based Odometry & Motion Control:** Utilizes an IMU and tracking wheels for accurate position tracking and path following.
* **Modular Drivetrain:** Configurable motor groups for left and right sides.
* **Autonomous Selector:** Allows selection of 10 different autonomous routines using a potentiometer.
* **Team Selector:** Simple switch/potentiometer input to select alliance color (Red/Blue), enabling adaptive autonomous paths.
* **Real-time Telemetry:** Displays robot pose (X, Y, Theta) directly on the V5 Brain screen.
* **Arcade Drive:** Standard and intuitive driver control scheme.

## 2. Setup and Installation

### Prerequisites
* **PROS Editor/CLI:** Ensure you have the PROS CLI installed and configured.
    * [PROS Installation Guide](https://pros.cs.purdue.edu/v5/getting-started/installation.html)
* **VS Code:** Recommended IDE for PROS development.
    * [VS Code Installation](https://code.visualstudio.com/download)
    * PROS VS Code Extension is highly recommended.
* **VEX V5 Robot Brain:** With updated firmware.

### Cloning the Repository
1.  Open a terminal or command prompt.
2.  Navigate to your desired directory.
3.  Clone the repository:
    ```bash
    git clone [https://github.com/Pahlaj-Sharma/2602K-KryptoKnights_Code_2025-2026.git](https://github.com/Pahlaj-Sharma/2602K-KryptoKnights_Code_2025-2026.git)
    cd 2602K-KryptoKnights_Code_2025-2026
    ```

### Building and Uploading
1.  **Open in VS Code:**
    ```bash
    code .
    ```
2.  **Initialize PROS Project (if necessary):**
    If this is a new clone and not yet a PROS project (e.g., if you only copied `src` and `include` folders), you may need to initialize it:
    ```bash
    pros conduct init
    ```
3.  **Install LemLib:**
    This project uses LemLib. Ensure it's installed as a PROS dependency:
    ```bash
    pros conduct install lemlib@0.4.7 # Or the latest stable version you are using
    ```
    *Note: If you encounter compiler errors related to C++11 features (like brace-enclosed initializer lists), you might need to manually add compiler flags to your `project.pros` file or `Makefile` (e.g., `-std=c++11` or `-std=gnu++11`).*
4.  **Build the Project:**
    ```bash
    pros make
    ```
5.  **Upload to V5 Brain:**
    Connect your V5 Brain to your computer via USB.
    ```bash
    pros upload
    ```

## 3. Hardware Overview

* **V5 Brain:** The central control unit.
* **V5 Motors:** Blue cartridge (600 RPM) motors for drivetrain.
    * Left Motors: Ports 14, 15, 16 (reversed)
    * Right Motors: Ports 11, 13, 12
* **V5 Inertial Sensor (IMU):** Port 2 (for odometry heading).
* **V5 Rotation Sensors:**
    * Horizontal Tracking Wheel: ADI Port -3 (reversed)
    * Vertical Tracking Wheel: ADI Port 17
* **V5 Potentiometers:**
    * Autonomous Selector: ADI Port 6
    * Team Selector: ADI Port 7 (used as a 2-state switch)
* **V5 Distance Sensors:** Ports 3 (ensure these are configured correctly if used for odometry resets or object detection).

## 4. Software Overview

The code is structured around standard PROS project conventions.

* **`src/main.cpp`**: The main application file.
    * **Motor & Sensor Definitions**: All hardware components are instantiated here.
    * **LemLib Configuration**: `Drivetrain`, `TrackingWheel`s, `OdomSensors`, `ControllerSettings` (PID tunings), and `Chassis` objects are defined for motion control.
    * **`initialize()`**: Called once when the robot starts. Handles motor brake modes, sensor resets, LCD initialization, and LemLib calibration. Also sets up a real-time screen telemetry task.
    * **`disabled()`**: Called when the robot is disabled. Good for resetting variables.
    * **`competition_initialize()`**: Runs after `initialize()` and before `autonomous()` or `opcontrol()`. This is where the autonomous and team selection logic runs, continuously updating the V5 Brain screen based on potentiometer readings.
    * **`autonomous()`**: Called when the robot is in autonomous mode. It selects and executes the chosen autonomous routine based on `autonSelect`.
    * **`opcontrol()`**: Called when the robot is in driver control mode. Implements arcade drive using controller joysticks.
    * **`autons` namespace**: Contains individual autonomous routine functions (e.g., `autons::auton1()`).

## 5. Autonomous Selection and Modes

The robot features two main selectors managed in `competition_initialize()`:

### Autonomous Selector
* Uses a potentiometer on **ADI Port 6**.
* The potentiometer's 0-330 range is divided into 10 segments, each corresponding to an autonomous routine from 1 to 10.
* Turn the potentiometer to select the desired autonomous routine before the match starts. The selected routine name will be displayed on the V5 Brain screen.

### Team Selector
* Uses a potentiometer on **ADI Port 7**.
* This potentiometer is treated as a simple switch:
    * `0-165` degrees: "RED" team.
    * `166-330` degrees: "BLUE" team.
* The selected team color is displayed on the V5 Brain screen. Your autonomous routines can use this `teamtype` string (or the `selectedTeam` integer) to adapt their paths for the red or blue alliance.

## 6. Driver Control

* **Arcade Drive:**
    * **Left Joystick (Y-axis):** Controls forward and backward movement.
    * **Right Joystick (X-axis):** Controls turning.

## 7. Troubleshooting and Notes

* **"C++11 standard" / "brace-enclosed initializer list" errors:** If you encounter errors related to C++11 features (specifically with `chassis.moveToPose` parameters like `{ .lead = 0.7, .minSpeed = 60 }`), ensure your PROS project is configured to use a C++ standard of C++11 or newer. You might need to add `-std=c++11` or `-std=gnu++11` to your compiler flags in `project.pros` or `Makefile`.
    * **Workaround for LemLib `moveToPose`:** If updating compiler flags doesn't work, you may need to explicitly create the parameter struct:
        ```cpp
        lemlib::MoveToPointParams params = {.lead = 0.7, .minSpeed = 60};
        chassis.moveToPose(45, 24, 90, 4500, params);
        ```
* **Potentiometer Calibration:** The exact analog values for your potentiometer may vary slightly. Test your potentiometer's full range to fine-tune the thresholds for autonomous and team selection.
* **IMU Calibration:** The `chassis.calibrate()` function will hold the robot still for a few seconds during `initialize()`. Ensure the robot is stationary during this process for accurate odometry.
* **Git Issues:** If you encounter `non-fast-forward` or `divergent branches` errors when pushing to GitHub, remember to `git pull origin main` *before* `git push origin main` to synchronize your local and remote branches.

## 8. Contributors

* Pahlaj Sharma (psharma1)

---