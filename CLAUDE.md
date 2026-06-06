# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Deploy Commands

```bash
./gradlew build          # Compile and check for errors (no robot needed)
./gradlew test           # Run JUnit 5 tests
./gradlew deploy         # Build and deploy to connected RoboRIO over USB or WiFi
./gradlew simulateJava   # Run WPILib desktop simulation with GUI
```

There is no linting step. `./gradlew build` is the primary correctness check — fix all compiler errors before deploying.

---

## Repository Context

This is a WPILib command-based Java robot for FRC Team YCS Robotics, 2026 season. The robot shoots fuel into a hub, intakes from the floor, and hangs. Vision localization drives an aim-and-shoot feature.

**Git tags:**
- `baseline-2026-cc` — original WCP Swerve CC import
- `season-2026-final` — code as it ran at competition (MISAL Q74, March 29 2026)
- `offseason-baseline-2026` — competition code with known localization bugs fixed; offseason starting point

---

## Architecture

### Two CAN buses
All drivetrain hardware (swerve modules, Pigeon2 gyro) runs on a **CANivore** named `"Swerve Bus"`. All mechanism hardware (intake, feeder, shooter, hanger) runs on the **RoboRIO** built-in CAN bus (`"rio"`). CAN IDs for mechanisms start at 50.

### Mechanism presence flags — `Constants.MechanismPresence`
Every mechanism subsystem checks its presence flag before creating hardware. Set `kSwerveOnlyBringup = true` to disable all mechanisms for drivetrain-only bringup. Otherwise, toggle individual flags (`kIntakePivotPresent`, `kShooterPresent`, etc.) to bring up mechanisms one at a time without CAN timeout errors from absent devices.

### Swerve and pose estimation — `Swerve.java`
Extends CTRE's `TunerSwerveDrivetrain` (generated in `TunerConstants.java`). The built-in `SwerveDrivePoseEstimator` (Kalman filter) fuses wheel odometry, Pigeon2 gyro, and vision measurements.

**Critical:** `getPigeon2().getYaw()` is an **unbounded accumulating register** — it grows past ±180° as the robot rotates. Never pass it to `SetRobotOrientation` or use it as a heading. Always use `currentRobotPose.getRotation().getDegrees()`, which is bounded to ±180° by `Rotation2d`.

### Vision localization — `Limelight.java`
Uses a single Limelight 4 with MegaTag2 (orientation-constrained XY solve). Runs as the default command on the `Limelight` subsystem (`visionUpdateCommand`), so it continues during teleop. During Choreo auto segments it must be explicitly scheduled via `limelight.idle(swerve)`.

**Fusion flow each loop:**
1. Send bounded heading to Limelight via `SetRobotOrientation` — MegaTag2 uses this to constrain its XY solve
2. Fetch MegaTag1 (pure vision, gives rotation) and MegaTag2 (heading-constrained, gives XY)
3. Compute yaw delta between MegaTag1 rotation and odometry rotation
4. If within 7.5° band: fuse MegaTag2 XY + MegaTag1 rotation into the pose estimator
5. If outside band: fuse MegaTag2 XY only, keep odometry heading (translation-only mode)
6. XY stddev scales dynamically: `max(0.05, 0.02 × dist² / tagCount²)` — distant single-tag estimates get low trust automatically

**Pose initialization:** On boot, `getMeasurement()` uses MegaTag1 (not MegaTag2) to initialize the pose, requiring 2+ tags. If vision never initializes, teleop start resets to a fallback pose based on alliance (Blue: x=3, y=4, 0°; Red: x=13, y=4, 180°).

### Aim-and-shoot — `AimAndDriveCommand` + `PrepareShotCommand` + `SubsystemCommands`
`AimAndDriveCommand` uses CTRE's `FieldCentricFacingAngle` request, always in **BlueAlliance** forward perspective regardless of actual alliance. It rotates the robot to face `Landmarks.hubPosition()`, which returns the correct hub (blue or red) based on the Driver Station alliance.

`PrepareShotCommand` uses an `InterpolatingTreeMap` (shot table) to look up shooter RPM and hood position from distance-to-hub. Distance is measured center-to-center (robot center to hub center), not bumper-to-hub-face.

`SubsystemCommands` is a **command factory, not a subsystem** — it holds no hardware, just composes commands from subsystems passed in at construction. `aimAndShoot()` fires when aimed + shooter at speed + translation stopped (not full stop — checking omega would fight the heading PID).

### Autonomous — `AutoRoutines.java` + Choreo trajectories
Trajectories live in `src/main/deploy/choreo/` as `.traj` files. They are loaded by name at runtime (no code generation required). Multi-segment trajectories use integer segment indices (0, 1, 2…). The auto chooser is published to SmartDashboard.

During Choreo segments, the Limelight default command is preempted by the trajectory follower. Any segment that should keep vision running must explicitly call `limelight.idle(swerve)` with `.whileTrue()`.

### Alliance handling — `Landmarks.java`
`Landmarks.hubPosition()` and `Landmarks.hubPosition()` are alliance-aware and read from the Driver Station at call time. When FMS is not attached, both default to Blue. Use `Landmarks.practiceRedHubPose()` for a fixed Red-side practice placement that bypasses the DS alliance check.

---

## Offseason Goals (through September 2026)

1. **Understand the baseline** — develop deep familiarity with every subsystem and command
2. **Localization and aim-and-shoot improvements** — the `offseason-baseline-2026` tag marks the starting point; remaining known issues are tracked as GitHub Issues
3. **Student engagement** — build a framework for high school students with limited coding experience to understand and contribute to the code using Claude Code

### Known localization issues (from MISAL Q74 log analysis)
Full analysis: [docs/analysis/MISAL_Q74_localization_analysis.md](docs/analysis/MISAL_Q74_localization_analysis.md)
- **Fixed:** Unbounded Pigeon2 yaw fed to `SetRobotOrientation` (raw value reached 1820°). Fixed in `26a0897`.
- **Fixed:** Flat XY stddev (0.1m) trusted distant single-tag estimates equally to close multi-tag estimates. Fixed in `19b0466`.
- **Open:** No field boundary validation — a bad MegaTag estimate could inject an out-of-field pose without rejection.
- **Open:** No camera health/disconnect monitoring during matches.
- **Open:** Odometry stddev in `Swerve` constructor (`VecBuilder.fill(0.1, 0.1, 0.1)`) has not been tuned relative to the new dynamic vision stddev.
- **Open:** `kVisionXYStdDevCoefficient = 0.02` is a starting point and needs empirical tuning against match logs.
