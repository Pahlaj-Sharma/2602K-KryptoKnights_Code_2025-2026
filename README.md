# 2602K KryptoKnights Robot Code (2025-2026)

This repository holds the C++ code for VEX V5 Robotics Team 2602K's robot for the 2025-2026 game, Push Back. It is a PROS project that contains two things:

1. **pahlib**, a motion control and localization library for differential (tank) drivetrains. It handles odometry, PID movements, boomerang pose-to-pose motion, swing turns, a Ramsete path follower, driver control curves, and a distance-sensor position correction system called RCL.
2. **The 2602K robot program** built on top of pahlib: the port map, subsystems, driver controls, ten autonomous routines, and an autonomous selector.

It also includes **2DMP**, a browser-based path planner that exports paths for pahlib's Ramsete controller and can generate `moveTo()` calls for you.

If you only want to use the library on your own robot, skip to [Setting up pahlib on your robot](#setting-up-pahlib-on-your-robot).

---

## Project layout

| Path | What it contains |
| --- | --- |
| `include/pahlib/` | pahlib headers. `api.hpp` includes everything you normally need. |
| `include/pahlib/chassis/` | `Chassis`, odometry and tracking wheel headers. |
| `src/pahlib/` | pahlib source: PID, exit conditions, drive curves, pose math, utilities, RCL tracking. |
| `src/pahlib/chassis/` | Chassis setup, odometry loop, driver control functions. |
| `src/pahlib/chassis/motions/` | One file per motion: `moveTo`, `turnTo`, `swingTo`, `ramsete`, plus the disabled `pursuit` and `stanley` files. |
| `include/robot_config.hpp` | Ports, drivetrain constants, PID presets, gain schedules and distance sensor offsets. pahlib reads from this file. |
| `include/main.h` | Global declarations and the `ScalarIMU` class. |
| `src/main.cpp` | Device definitions, chassis construction, RCL setup, auton selector, driver control. |
| `src/autons.cpp` | The ten autonomous routines. |
| `src/subsystems.cpp` | Intake and scoring motor helpers. |
| `2DMP/` | The path planner (`index.html`) and field images. |
| `static/` | Asset files (paths) that can be compiled into the program. |
| `include/fmt/`, `include/units/` | Third-party headers used by pahlib (fmt for string formatting, a units library carried over from LemLib). |
| `firmware/` | The PROS kernel libraries and linker scripts. |
| `src/pahlib/*.txt` | Archived experiments that are not compiled: a Monte Carlo localization (particle filter) attempt and two earlier distance-sensor localization versions. |

---

## Requirements

- A VEX V5 Brain and controller.
- PROS 4. The project was created with kernel 4.2.2 (see `project.pros`). The easiest way to get PROS and its ARM toolchain is the PROS extension for VS Code. The standalone PROS CLI also works.
- For the full 2602K program: six drive motors, an intake motor, a scoring motor, an IMU, two rotation sensors on tracking wheels, four distance sensors, pneumatics, and a button for the auton selector. pahlib itself only needs the drive motors; everything else is optional.

## Building and uploading

1. Clone the repository:
   ```bash
   git clone https://github.com/Pahlaj-Sharma/2602K-KryptoKnights_Code_2025-2026.git
   ```
2. Open the folder in VS Code with the PROS extension installed (or `cd` into it if you use the CLI).
3. Build the project:
   ```bash
   pros make
   ```
4. Connect the Brain or controller over USB and upload:
   ```bash
   pros upload
   ```
   `pros mu` does both steps. The program uploads to slot 1 as "KryptoKnights Code" (set in `project.pros`).
5. To see `printf` output from the robot, run `pros terminal`.

---

## How pahlib works

### Coordinates and units

- Distances are in inches.
- Headings are in degrees using the VEX compass convention: 0° points along +Y, 90° points along +X, and angles increase clockwise. This matches what the IMU reports.
- The 2602K code and the RCL system put the origin at the center of the field, so the field walls sit at ±70.5 inches.
- Motor power is on the -127 to 127 scale used by `pros::Motor::move()`.

`chassis.getPose()` returns a `pahlib::Pose` with `x`, `y` and `theta`. Pass `true` to get `theta` in radians, and `true` as a second argument to get it in standard math position (0 along +X, counterclockwise positive).

### Odometry

Calling `chassis.calibrate()` in `initialize()` does three things. It calibrates the IMU, retrying up to five times and rumbling the controller ("---") on each failed attempt; if all five fail, odometry continues without the IMU. It fills in missing vertical tracking wheels using the drive motors, placed at plus and minus half the track width. Finally it starts a background task that updates the robot's position every 10 ms and rumbles once (".") when everything is ready.

Each update reads the change in every tracking wheel and the IMU. Heading comes from the best available source, in this order: two horizontal tracking wheels, two unpowered vertical tracking wheels, the IMU, and finally the drivetrain motors. The change in position is then calculated as an arc in the robot's local frame and rotated into field coordinates using the average heading over that 10 ms step. The task also keeps smoothed estimates of global and local velocity.

A `TrackingWheel` can be built from a V5 rotation sensor, an ADI (three-wire) encoder, or a motor group:

```cpp
pahlib::TrackingWheel(pros::Rotation* sensor, float wheelDiameter, float offset, float gearRatio = 1);
pahlib::TrackingWheel(pros::adi::Encoder* encoder, float wheelDiameter, float offset, float gearRatio = 1);
pahlib::TrackingWheel(pros::MotorGroup* motors, float wheelDiameter, float offset, float rpm);
```

`offset` is the distance from the robot's tracking center to the wheel. For vertical wheels, negative means left of center and positive means right. For horizontal wheels, positive means in front of center and negative means behind. If the robot's X or Y value drifts when it spins in place, the offset or the sensor direction is wrong.

The `pahlib::Omniwheel` namespace has diameter constants for standard VEX omni wheels: `NEW_2`, `NEW_275`, `OLD_275`, `NEW_275_HALF`, `OLD_275_HALF`, `NEW_325`, `OLD_325`, `NEW_325_HALF`, `OLD_325_HALF`, `NEW_4`, `OLD_4`, `NEW_4_HALF` and `OLD_4_HALF`. For tracking wheels, measuring your actual wheel is more accurate. The 2602K robot uses measured values of 1.9657" and 1.9705" for its 2" wheels.

#### ScalarIMU

`main.h` defines `ScalarIMU`, a subclass of `pros::IMU` that multiplies the reported rotation and heading by a correction factor. Every IMU has a small scale error, and it adds up over a long skills run. To find your factor, turn the robot exactly 10 full rotations by hand, read the IMU's rotation value, and divide 3600 by that number. The 2602K robot uses 1.00912. Pass a `ScalarIMU` to `OdomSensors` the same way you would pass a normal IMU.

### The Chassis and the motion queue

`pahlib::Chassis` is built from four pieces:

- `Drivetrain`: the left and right motor groups, track width, drive wheel diameter, drivetrain RPM, and horizontal drift (explained under boomerang below).
- Two `ControllerSettings`: one for lateral (driving) and one for angular (turning) control.
- `OdomSensors`: up to two vertical wheels, two horizontal wheels, and an IMU. Pass `nullptr` for anything you don't have.
- Optional throttle and steer `DriveCurve`s for driver control.

`ControllerSettings` takes ten values in this order:

| # | Field | Meaning |
| --- | --- | --- |
| 1 | `kP` | Proportional gain. |
| 2 | `kI` | Integral gain. Set to 0 to disable. |
| 3 | `kD` | Derivative gain. |
| 4 | `kF` | Feedforward gain. Stored, but the current PID output does not use it. |
| 5 | `windupRange` | The integral resets to zero whenever the error is larger than this. 0 disables the limit. |
| 6 | `smallError` | Small exit range (inches or degrees). |
| 7 | `smallErrorTimeout` | How long, in ms, the error must stay inside the small range before the motion ends. |
| 8 | `largeError` | Large exit range. |
| 9 | `largeErrorTimeout` | How long the error must stay inside the large range before the motion ends. |
| 10 | `slew` | The largest change in motor power allowed per 10 ms loop. 0 disables slew. |

Every motion function is asynchronous by default. The call starts a background task and returns right away, so your code can run mechanisms while the robot drives. If you call another motion while one is running, the new one waits its turn: motions are queued with a mutex and run in order. These functions let you coordinate with a running motion:

- `chassis.waitUntil(dist)` blocks until the robot has traveled `dist` along the current motion. The units are inches for `moveTo` and `ramsete`, and degrees for turns and swings.
- `chassis.waitUntilDone()` blocks until the current motion finishes.
- `chassis.cancelMotion()` stops the current motion, and `chassis.cancelAllMotions()` also clears anything queued.
- `chassis.isInMotion()` reports whether a motion is running.

Pass `false` as the final `async` argument to make a single call blocking instead.

### Exit conditions

A motion ends when any of these happens:

- The error stays inside `smallError` for `smallErrorTimeout` ms.
- The error stays inside `largeError` for `largeErrorTimeout` ms.
- The motion's `timeout` runs out.
- It is cancelled.

The small range gives you a precise finish. The large range stops the robot from spending seconds creeping toward a target it has almost reached.

### Motion chaining

Setting a non-zero `minSpeed` on a motion changes how it ends. Instead of slowing to a stop, the robot keeps at least `minSpeed` and exits as soon as it crosses the line through the target that is perpendicular to its path. `earlyExitRange` moves that line toward the robot, so the motion ends that many inches (or degrees, for turns) before the target. The next motion in the queue picks up without the robot stopping, which makes routines faster and smoother. The 2602K autons use this constantly, for example:

```cpp
chassis.moveTo(-32, 47, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});
```

### Motions

Motion options are passed as a struct with named fields, so you only write the ones you want to change: `{.maxSpeed = 60, .minSpeed = 20}`. Fields must appear in the order they are declared in the struct.

#### `moveTo(x, y, timeout, params)`: drive to a point

The robot turns toward the point while driving to it. The lateral PID works on the distance to the target multiplied by the cosine of the heading error, so the robot drives slower when it isn't facing the point. The angular PID works on the heading error to the point. Once the robot is within 4 inches, it stops steering, caps its speed, and settles on distance alone, which avoids spinning in circles around the target.

| Param | Default | Meaning |
| --- | --- | --- |
| `forwards` | `true` | Drive with the front (`true`) or back (`false`) of the robot. |
| `maxSpeed` | 127 | Top speed. |
| `minSpeed` | 0 | See motion chaining. |
| `earlyExitRange` | 0 | See motion chaining. |
| `maxAcceleration` | 60 | Reserved. Acceleration is limited by `slew` in `ControllerSettings`. |
| `gainScheduling` | `false` | Reserved for the point version (only the pose version uses it). |

#### `moveTo(x, y, theta, timeout, params)`: drive to a pose (boomerang)

Adding a heading turns `moveTo` into a boomerang controller, which drives to the point and finishes facing `theta`. Rather than aiming straight at the target, the robot chases a "carrot" point placed behind the target along the final heading:

```
carrot = target - (cos θ, sin θ) × lead × distanceToTarget
```

As the robot gets closer, the carrot slides onto the target, so the path curves in and ends at the right heading. A higher `lead` gives a wider curve.

Two limits keep the robot under control on curves. The lateral speed is capped at `sqrt(horizontalDrift × turnRadius × 9.8)`, the fastest the robot can take that curve without sliding. Turning also gets priority over driving: if the combined output would exceed `maxSpeed`, lateral power is cut first. Within 4 inches the robot switches to targeting the final point and heading directly.

| Param | Default | Meaning |
| --- | --- | --- |
| `forwards` | `true` | Drive forwards or backwards. |
| `horizontalDrift` | 0 | Overrides the drivetrain's horizontal drift for this motion. 0 uses the drivetrain value. Around 2 suits omni wheels and around 8 suits traction wheels. |
| `lead` | 0.6 | Carrot distance multiplier, from 0 to 1. Use 0 for an almost straight approach. |
| `maxSpeed` | 127 | Top speed. |
| `minSpeed` | 0 | See motion chaining. |
| `earlyExitRange` | 0 | See motion chaining. |
| `settleDist` | 4 | Reserved. The settle distance is currently fixed at 4 inches. |
| `maxAcceleration` | 60 | Reserved. |
| `gainScheduling` | `false` | Blend PID gains by error while settling (see below). |

#### `turnTo(theta, timeout, params)` and `turnTo(x, y, timeout, params)`: turn in place

The first version turns to a heading and the second turns to face a point. `params.direction` can be `AngularDirection::AUTO` (shortest way, the default), `CW_CLOCKWISE` or `CCW_COUNTERCLOCKWISE`. A forced direction only applies until the robot first crosses the target; after that it corrects the shortest way. Slew is applied only while more than 20° away, so the final correction stays sharp. The point version also takes `forwards = false` to face the point with the back of the robot. Turns finish with the drive motors in hold mode. Other params: `maxSpeed`, `minSpeed`, `earlyExitRange`.

#### `swingTo(theta, side, timeout, params)` and `swingTo(x, y, side, timeout, params)`: swing turn

A swing turn locks one side of the drivetrain in hold mode and pivots on it by driving only the other side. `side` is `DriveSide::LEFT` or `DriveSide::RIGHT`, the side that stays still. The locked side's brake mode is restored afterward. Params match `turnTo`.

#### `moveLinear(inches, timeout, lead, maxSpeed, minSpeed)`: straight drive

A shortcut that drives a set distance forward (or backward, with a negative number) along the robot's current heading. It calculates a target pose and passes it to the boomerang `moveTo`. Defaults: 2000 ms timeout, lead 0.1, max speed 70, min speed 40.

#### `ramsete(path, beta, zeta, timeout)`: path following

Follows a path made in 2DMP using a Ramsete controller. Each path point holds x, y, heading, and a target velocity on the motor power scale. Every loop, the controller:

1. Finds the closest path point, then looks 6 inches ahead along the path for a target point.
2. Takes that point's velocity as the target forward speed, and estimates the target turning rate from the change in heading to the next point.
3. Converts the position error into the robot's frame and applies the Ramsete control law, which corrects cross-track and heading error in a way that settles back onto the path.
4. Turns the forward and turning commands into left and right wheel power, scales them to stay under 127, and applies slew.

The motion ends within 2 inches of the last point or when the timeout runs out. `beta` sets how hard the robot corrects position error, and `zeta` adds damping. WPILib's defaults (β = 2.0, ζ = 0.7) are a reasonable starting point, but expect to tune them, since pahlib's velocities are in motor power units, not meters per second. Driving direction comes from the path file: reversed segments have negative velocities.

To load a path, put the file in `static/` and declare it at the top of a source file, outside any function:

```cpp
ASSET(skills_txt); // loads static/skills.txt (the "." becomes "_")

void autonomous() {
    chassis.setPose(-46, 14.5, 90);
    chassis.ramsete(skills_txt, 2.0, 0.7, 5000);
}
```

Files in `static/` are only compiled into the program if the project has a makefile rule that embeds them. This repository doesn't include one yet. Copy `firmware/hot-cold-asset.mk` from LemLib's project template into `firmware/`; the existing `common.mk` picks up any `.mk` file in that folder automatically. The current `static/path.txt` is in path.jerryio format (x, y, speed), not the 2DMP format that `ramsete()` reads.

### PID presets and gain scheduling

`robot_config.hpp` defines three sets of PID constants, normal, fast and precise, for both lateral and angular control. Switch between them mid-routine:

```cpp
chassis.setPID(pahlib::Chassis::PIDPreset::fast);
chassis.setPID(pahlib::Chassis::PIDPreset::precise);
chassis.setPID(pahlib::Chassis::PIDPreset::normal);
```

Or set gains directly: `chassis.setPID(latP, latI, latD, latF, angP, angI, angD, angF)`. Either call replaces the gains used by every motion after it. The exit conditions from `ControllerSettings` stay the same.

With `.gainScheduling = true`, the boomerang `moveTo` blends between gain sets once it is within 4 inches of the target. `LateralSchedule` in `robot_config.hpp` lists gains for errors of 2, 5, 10 and 20 inches, and `AngularSchedule` lists gains for 15°, 45°, 90° and 180°. The code interpolates linearly between the two nearest entries. This lets you use soft gains for tiny corrections and stronger gains for larger ones.

### Driver control and drive curves

The Chassis has three driver control modes. All of them take joystick values from -127 to 127:

```cpp
chassis.tank(leftY, rightY);
chassis.arcade(leftY, rightX);
chassis.curvature(leftY, rightX);
```

`arcade` has a fourth argument, `desaturateBias` (default 0.5). When the throttle and turn inputs add up to more than 127, it decides which one gets reduced: 0 keeps full throttle, 1 keeps full turning. `curvature` makes the turn input control the radius of the curve instead of the turning speed, so steering feels the same at any speed. It falls back to arcade when the throttle is zero.

Pass `true` as the third argument to skip the drive curves and send raw power. The 2602K code does this for scripted pushes like `chassis.tank(-40, -40, true)`.

`ExpoDriveCurve(deadband, minOutput, curve)` reshapes stick input:

- Inputs within `deadband` return 0, which hides stick drift.
- Just outside the deadband, the output jumps to `minOutput`, enough power to overcome friction.
- `curve` above 1 makes the response exponential, giving finer control near the center of the stick while still reaching 127 at full stick. A value of 1 is linear.

The 2602K robot uses `ExpoDriveCurve(5, 25, 1.05)` for throttle and `ExpoDriveCurve(5, 5, 1.01)` for steering.

### RCL: distance sensor position correction

Odometry drifts over time, especially after contact with other robots or field elements. RCL (Relative Coordinate Localization) uses V5 distance sensors pointed at the field walls to correct the robot's X and Y position while it moves.

Each `RclSensor` describes one distance sensor:

```cpp
RclSensor(pros::Distance* sensor, double xOffset, double yOffset, double mountAngle, double angleTolerance = 10.0);
```

`xOffset` and `yOffset` are the sensor's position relative to the robot's center (right and forward are positive). `mountAngle` is the direction the sensor faces relative to the robot's front: 0 for front, 90 for right, 180 for back, 270 for left. Sensors register themselves when they are created.

Every tick, the main loop works through each sensor:

1. It uses the current pose to work out where the sensor is on the field and which wall its beam is pointed at.
2. It rejects the reading if the distance is over 2000 mm, if the sensor's confidence is below 60 at distances over 200 mm, if the beam is not within `angleTolerance` degrees of square to a wall, or if the beam passes through a known obstacle.
3. It converts the reading into the robot's X coordinate (east or west walls) or Y coordinate (north or south walls), corrected for the sensor's mounting offset.
4. It discards any value more than `maxDelta` inches from the current estimate, then averages what is left. The averaged value is kept only if it differs from the current estimate by at least `minDelta`.

Obstacles are field elements that would block a sensor's view of the wall. `Circle_Obstacle(x, y, radius)` and `Line_Obstacle(x1, y1, x2, y2)` register themselves on creation, and `Line_Obstacle::addPolygonObstacle({{x, y}, ...})` builds a closed shape from lines. The 2602K code marks the four loaders, the four long goal ends, and the center goals, and adds a line down the middle of the field. Obstacles also accept a lifetime in milliseconds, but the cleanup call in `miscLoop()` is commented out, so for now they last forever.

`RclTracking` runs the system:

```cpp
RclTracking(pahlib::Chassis* chassis,
            int frequencyHz = 25,       // how often to read sensors
            bool autoSync = true,       // gradually pull the chassis pose toward the RCL pose
            double minDelta = 0.5,      // ignore corrections smaller than this (in)
            double maxDelta = 4.0,      // ignore readings further than this from the estimate (in)
            double maxDeltaFrompahlib = 10.0, // if the RCL pose is further than this from odometry, pull it back (in)
            double maxSyncPerSec = 3.0, // how fast auto-sync is allowed to move the pose (in/s)
            int minPause = 20);         // minimum delay between loop iterations (ms)
```

With `autoSync` on, a second loop moves the chassis pose toward the RCL pose, limited to `maxSyncPerSec`. The robot's position is corrected smoothly instead of jumping, so a motion in progress doesn't jerk. Other useful calls:

| Call | What it does |
| --- | --- |
| `startTracking()` / `stopTracking()` | Start or stop the background loops. |
| `updateBotPose(&sensor)` | Instant reset of one axis from a single sensor. Good for squaring up against a wall. |
| `updateBotPose()` | Apply the full RCL pose to the chassis right away. |
| `setRclPose(pose)` | Set the RCL reference. Call `reset.setRclPose(chassis.getPose())` after every `chassis.setPose()` to clear any stale correction. |
| `accumulateFor(ms)` | Average readings over a period while the robot is still, then apply the result. More accurate than a single reading. |
| `startAccumulating()` / `stopAccumulating()` | The same, but you control when it starts and stops. |
| `discardData()` | Throw away the current correction and match the chassis pose. |
| `setMaxSyncPerSec(value)` | Change the auto-sync speed during a routine. |

---

## Setting up pahlib on your robot

### Option A: start from this repository

This is the quickest route, since everything is already wired together.

1. Clone the repo and confirm it builds (see [Building and uploading](#building-and-uploading)).
2. Open `include/robot_config.hpp` and change the ports, track width, drivetrain RPM, IMU scaler, tracking wheel offsets and distance sensor offsets to match your robot. A negative port number reverses that motor or sensor.
3. In `src/main.cpp`, check the motor gearsets (`MotorGears::blue` is 600 RPM), the drive wheel constant passed to `Drivetrain`, and the tracking wheel diameters. Remove any devices you don't have. Anything that isn't present should be `nullptr` in `OdomSensors`.
4. If you don't have distance sensors, delete the `RclSensor`, obstacle and `RclTracking` definitions along with the `reset.*` calls. Otherwise, update the sensors and the obstacles for your game.
5. Replace the subsystem code in `src/subsystems.cpp` and the button mappings in `opcontrol()` with your own mechanisms.
6. Tune your PID constants (see [Tuning](#tuning)).
7. Write your routines in `src/autons.cpp`, declare them in `include/autons.hpp`, and add them to the `autons` map in `main.cpp`.

### Option B: add pahlib to an existing PROS 4 project

1. Copy these folders into your project, keeping the same paths:
   - `include/pahlib/` and `src/pahlib/`
   - `include/fmt/` (used by `pose.cpp`)
   - `include/units/` (included by the motion files)
2. Copy `include/robot_config.hpp` into your `include/` folder. pahlib includes it directly, so it has to exist even if you ignore most of it. The library needs the `PIDConstants` struct, the six presets (`LATERAL_PID`, `F_LATERAL_PID`, `P_LATERAL_PID`, `ANGULAR_PID`, `F_ANGULAR_PID`, `P_ANGULAR_PID`), and the `LateralSchedule` and `AngularSchedule` structs. The port constants can be deleted if you define ports elsewhere.
3. Some pahlib files also include `main.h`. The default PROS `main.h` works.
4. If you want IMU scale correction, copy the `ScalarIMU` class out of this repo's `main.h`.
5. If you plan to use `ramsete()`, add the asset makefile described in the Ramsete section.
6. Set up your chassis:

```cpp
#include "main.h"
#include "pahlib/api.hpp"
#include "robot_config.hpp"

// drive motors (negative port = reversed)
pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGears::blue);
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGears::blue);

// sensors
pros::Rotation vertical_encoder(7);
pros::Rotation horizontal_encoder(8);
pros::Imu imu(9);

pahlib::Drivetrain drivetrain(&left_motors, &right_motors,
                              11.25,                      // track width (in)
                              pahlib::Omniwheel::NEW_325, // drive wheel diameter
                              450,                        // drivetrain RPM
                              2);                         // horizontal drift

pahlib::TrackingWheel vertical_wheel(&vertical_encoder, pahlib::Omniwheel::NEW_2, 0.0);
pahlib::TrackingWheel horizontal_wheel(&horizontal_encoder, pahlib::Omniwheel::NEW_2, -1.6875);

pahlib::OdomSensors sensors(&vertical_wheel,   // vertical wheel 1
                            nullptr,           // vertical wheel 2
                            &horizontal_wheel, // horizontal wheel 1
                            nullptr,           // horizontal wheel 2
                            &imu);

//                                   kP   kI  kD   kF  windup  small  ms   large  ms   slew
pahlib::ControllerSettings lateral(  9.5, 0,  9.5, 0,  3,      1,     100, 2,     500, 20);
pahlib::ControllerSettings angular(  4,   0,  25,  0,  3,      1,     100, 3,     500, 0);

pahlib::ExpoDriveCurve throttle_curve(5, 25, 1.05);
pahlib::ExpoDriveCurve steer_curve(5, 5, 1.01);

pahlib::Chassis chassis(drivetrain, lateral, angular, sensors, &throttle_curve, &steer_curve);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() {
    chassis.calibrate(); // calibrate the IMU and start odometry
}

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveTo(0, 24, 2000);                 // drive 24" forward
    chassis.turnTo(90, 1000);                    // face +X
    chassis.moveTo(24, 48, 0, 3000, {.lead = 0.4});
    chassis.waitUntilDone();
}

void opcontrol() {
    while (true) {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);
        pros::delay(10);
    }
}
```

A note on names: `RclTracking.hpp` declares its own global `Timer` and `degToRad` alongside `pahlib::Timer` and `pahlib::degToRad`. If you write `using namespace pahlib;` and then use either name unqualified, the compiler may report it as ambiguous. Write `pahlib::Timer` explicitly to avoid this.

### Tuning

Tune the angular controller first, since the lateral controller depends on the robot holding its heading.

1. Set `kI` and `kD` to 0 and `kP` to a small value. Run a 90° `turnTo`.
2. Raise `kP` until the robot overshoots and oscillates around the target.
3. Raise `kD` until the oscillation stops and the robot settles cleanly. If it now stops short, go back and raise `kP` a little.
4. Only add `kI` if the robot consistently stops a degree or two short. Keep `windupRange` small so the integral only acts near the target.
5. Repeat with `moveTo` for the lateral controller, using straight 24" and 48" drives.
6. Tighten `smallError` and `largeError` for accuracy, or loosen them and shorten the timeouts for speed.

Check odometry before tuning anything: push the robot around by hand with the pose printed on the Brain screen (the 2602K `initialize()` already does this during autonomous) and confirm the numbers match what you measure on the field.

---

## 2DMP path creator

`2DMP/index.html` is a standalone path planner. Open it in any web browser; it runs locally and loads `field.png` from the same folder.

- Add points by clicking on the field. Drag a point to move it. The path between points is a Catmull-Rom spline, a smooth curve that passes through every point.
- Edit a point in the side panel: X, Y, heading, and a target velocity cap for that waypoint. The ↺ and ↻ buttons rotate the heading by 90°.
- Reverse a segment by clicking on the path between two points. That segment is then driven backwards.
- Settings: track width, maximum velocity (`V_max`), and acceleration distance.
- Simulate (▶) animates a robot along the path at the planned speeds.
- Flip (↔ and ↕) mirrors the path for the other side of the field.
- Download (⬇) saves `_path.txt` for `ramsete()`. Rename it, put it in `static/`, and load it with `ASSET()`.
- Copy moveTo() (⧉) copies a list of boomerang `moveTo()` calls, one per waypoint, with estimated timeouts. Paste them into an auton as a starting point.
- Load (↑) reopens a saved path file for editing.

The velocity at each point along the path is the lowest of three limits, with a floor of 20:

1. A curvature limit, which slows the outer wheel's speed on tight curves using `V_max × r / (r + trackWidth/2)`.
2. The waypoint caps you set, blended between waypoints at a constant acceleration.
3. A ramp that speeds up over the first "acceleration distance" inches and slows down over the last.

The exported file looks like this, with one `x, y, heading, velocity` group per point (reversed segments have negative velocity):

```
double followPath[][] = {{
    -46.000, 14.500, 90.00, 20.00}, {
    -45.012, 14.531, 89.12, 24.87}, {
    ...
    -10.000, 47.000, 270.00, 20.00}}
};
```

---

## The 2602K robot program

### Autonomous routines and selector

A button on ADI port E (`PORT_AUTON_SELECTOR`) cycles through the routines while the robot is disabled on the field. The Brain screen shows the selected routine's name and where to place the robot. The controller shows the hottest drive motor temperature, the hottest intake motor temperature, and the selected auton number.

| # | Routine | Setup (as shown on the Brain) |
| --- | --- | --- |
| 1 | Skills | Left of Park; Right Side DT; Facing 90 |
| 2 | Counter SAWP | Middle of Park; Right Side DT; Facing 180 |
| 3 | Fast Right 4 | Right of Park; Left Side DT; Facing 90 |
| 4 | Right 7 | Right of Park; Left Side DT; Facing 90 |
| 5 | Right 9 Tech | Right of Park; Left Side DT; Facing 90 |
| 6 | Right 9 Split | Right of Park; Left Side DT; Facing 90 |
| 7 | Fast Left 4 | Left of Park; Right Side DT; Facing 90 |
| 8 | Left 7 | Left of Park; Right Side DT; Facing 90 |
| 9 | Left 7 Split | Left of Park; Right Side DT; Facing 180 |
| 10 | Left 7 Center First | Left of Park; Right Side DT; Facing 90 |

`initialize()` also checks every device and prints the name and port of anything that isn't connected, so a loose cable shows up before the match.

### Driver controls

| Input | Action |
| --- | --- |
| Left stick Y, right stick X | Arcade drive with the expo curves |
| Y | Toggle the intake preroller |
| R1 (hold) | Score: runs the scoring rollers and fires the score piston |
| L1 (hold) | Score in the middle goal. For the first 200 ms the rollers run in reverse, then the center goal piston fires and the rollers switch to scoring speed. |
| L2 (hold) | Score in the low goal |
| R2 | Toggle the antenna piston |
| A | Toggle the descore piston |
| Right arrow | Toggle the match load piston |
| Up arrow | Automated match loader sequence |
| Down arrow | Automated "clear bottom" sequence (the loading sequence with the drive arc mirrored) |

RCL tracking stops when driver control begins.

---

## Current status

pahlib is under active development, and a few pieces are unfinished. If you are using the library, know about these before you rely on them:

- `follow()` (pure pursuit) has its body commented out and currently does nothing. Use `ramsete()` for path following.
- The Stanley controller in `stanley.cpp` is commented out.
- `pahlaj()`, `autoTunePID()` and `resetOdometry()` are declared in `chassis.hpp` but not implemented. Calling any of them causes a linker error.
- The optional `PIDGains` arguments on `moveTo`, `turnTo` and `swingTo` are accepted but not applied yet. On the boomerang `moveTo`, passing them only turns off gain scheduling. Use `setPID()` to change gains for now.
- The `kF` term is stored but not used in the PID output. The trapezoidal `MotionProfile` class exists, but the motions don't call it yet.
- Some example code in the header comments still uses LemLib function names (`moveToPose`, `turnToHeading` and so on). The real names are `moveTo`, `turnTo` and `swingTo`, and the examples in this README use them.

---

## Copyright and license

Copyright 2025-2026 Pahlaj Sharma. All rights reserved.

This project is currently provided with all rights reserved by the author. No unauthorized reproduction, distribution, or modification is permitted without explicit written consent.

- Project lead: Pahlaj Sharma
- Team: 2602K KryptoKnights
- Current software version: 4.01
- Last updated: March 2, 2026

---

Inspired by [LemLib](https://github.com/LemLib/LemLib).
