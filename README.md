# Zombie Ricky Swerve (2026 Offseason)

Drivetrain-only robot code for **Zombie Ricky**, Team 4295's 2025 REEFSCAPE robot brought back for the offseason. It is a driver-practice robot and a YAGSL swerve baseline for programmers.

- **Hardware:** 4× SDS MK4i L3 modules, 8× NEO on SPARK MAX, 4× CTRE CANcoder, navX2 on the roboRIO MXP port.
- **Software:** WPILib 2026.2.1 (Java, command-based) + [YAGSL](https://github.com/Yet-Another-Software-Suite/YAGSL) **2026.4.1**.

## ⚠️ YAGSL is pinned to 2026.4.1. Do not update it.

YAGSL was rewritten in 2026.8.05. The new version uses a different JSON format, no longer supports the navX2 on the MXP port, and has a different Java API. **If VS Code offers to update the YAGSL vendordep, say no.**

`vendordeps/yagsl-2026.4.1.json` has its `jsonUrl` pointed at the frozen 2026.4.1 file, so a normal "check for updates" should find nothing new.

Docs that match this version:
- The example project at tag 2026.4.1: <https://github.com/Yet-Another-Software-Suite/YAGSL/tree/2026.4.1/examples/drivebase_only_2026>
- This repo's code comments

The docs at yagsl.yassrobotics.com describe the **new** version.

## Build, simulate, deploy

Use the WPILib VS Code commands, or run these from a terminal:

```bash
./gradlew build          # compile
./gradlew simulateJava   # run in the simulator (GUI opens; plug in an Xbox controller)
./gradlew deploy         # deploy to the robot (team 4295)
```

If Gradle complains about Java, point it at the WPILib JDK: add `-Dorg.gradle.java.home=$HOME/wpilib/2026/jdk` to the command.

## Driver controls (Xbox, USB port 0)

| Input | Action |
|---|---|
| Left stick | Drive. Field-relative: "up" is away from the driver after zeroing. |
| Right stick X | Rotate |
| Right bumper (hold) | Slow mode (40% of the current speed cap) |
| Left bumper (hold) | X-lock the wheels |
| **Start** | Zero gyro. **Point the robot away from the driver first.** |
| Back | Toggle robot-relative, the fallback if the gyro is wrong |

The speed cap starts at **25%** on every boot. Raise it from the dashboard: `Drive/Speed Cap`, from 0 to 1.

## Dashboard (Elastic or AdvantageScope → NetworkTables → SmartDashboard)

| Key | What it is |
|---|---|
| `Health/All OK` | Green when no device errors. Details are in the **Alerts** widget. |
| `Drive/Speed Cap`, `Drive/Robot Relative`, `Drive/Slow Mode`, `Drive/Heading (deg)` | Driver state |
| `Drive/Zero Gyro` | Button, same as Start |
| `Bench/1 … 6` | Bench-test buttons. **Robot on blocks.** Click to start, click again to stop. |
| `Bench/<module>/Steer Angle (deg)` vs `Bench/Steer Setpoint (deg)` | Graph these to tune steer PID |
| `Bench/<module>/Drive Forward Speed (mps)` vs `Bench/Drive Setpoint (mps)` | Graph these to tune drive PID |
| `Tuning/Drive/*`, `Tuning/Steer/*` | Live PIDF gains; press `Tuning/Apply PIDF` to send them |
| `Tuning/Print PIDF JSON` | Prints the gains as JSON to the Driver Station console |
| `Tuning/Capture Offsets` | Disabled, wheels straight, bevels left: prints each module's `absoluteEncoderOffset` |
| `swerve/modules/frontleft/…` (etc.) | YAGSL's own per-module values: raw and adjusted CANcoder angle, setpoints |

For AdvantageScope's **Swerve** tab, use `SmartDashboard/swerve/measuredStates` and `desiredStates`, with rotation `SmartDashboard/swerve/robotRotation`.

Every run is recorded to a `.wpilog` file (on the roboRIO, or on a USB stick if one is plugged in). Open it in AdvantageScope to replay a bad run.

## Where the robot settings live

Settings live in the JSON files, not the Java: `src/main/deploy/swerve/zombie-ricky/`.

| File | Holds |
|---|---|
| `swervedrive.json` | Gyro type (`navx_spi`) and `invertedIMU` |
| `modules/frontleft.json` etc. | CAN IDs (drive, steer, CANcoder), inversions, `absoluteEncoderOffset`, location in inches from robot center |
| `modules/physicalproperties.json` | Gear ratios (steer 150/7, drive 6.12), wheel diameter, current limits |
| `modules/pidfproperties.json` | Drive and steer PIDF. One set of each, shared by all 4 modules. |

CAN IDs, as carried over from Ricky's 2025 config:

| Module | Drive | Steer | CANcoder |
|---|---|---|---|
| frontleft | 8 | 13 | 24 |
| frontright | 2 | 7 | 22 |
| backleft | 9 | 5 | 21 |
| backright | 3 | 4 | 23 |

The offsets and the 15 in module locations are 2025 values. **Re-capture and re-measure them before trusting them.**

## Quick troubleshooting

| Symptom | Look at |
|---|---|
| A SPARK MAX does nothing, no error | Duplicate CAN ID, often the same ID as the PDH |
| `CAN frame not received … cancoder NN` in the console, plus loop overruns | That CANcoder's ID, wiring or power |
| A module spins nonstop | That module's `inverted.angle` |
| Wheels point wrong at rest, or one drags | `absoluteEncoderOffset`: re-capture it |
| Robot spins cleanly but the wrong way | Drive inversions, or a front-left ↔ back-right swap |
| Fine at the start heading, wrong after turning | `invertedIMU` |
| Wheels wobble around their angle | Steer P too high; add D |
| Strafing slowly turns the robot | Steer PID or offsets. The simulator shows a little of this even when correct. |
