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

## Engineering Constraints for All Suggestions

The robot operates under two hard constraints that must be explicitly considered for every code or tuning change:

**1. Loop overrun budget — 20ms cycle time**
The robot runs a 20ms control loop. Loop overruns are already being observed. Any suggestion that adds work to `robotPeriodic()` or subsystem `periodic()` methods — including NetworkTables publishing, additional sensor reads, logging calls, or heavier computation — must flag the loop overrun risk and justify the cost. Prefer post-run log analysis (hoot / wpilog) over live publishing to avoid adding loop burden.

**2. Match time tradeoff — 2.5 minutes to score as many points as possible**
Every tuning decision involves a tradeoff between current draw (battery protection) and cycle time (how fast the robot can intake, drive, aim, and shoot). When proposing a change — current limits, slew rates, acceleration profiles, command sequencing — explicitly state:
- What it protects against (brownout, motor damage, etc.)
- What it costs in cycle time or responsiveness
- How to validate the tradeoff on the practice field

---

## Repository Context

This is a WPILib command-based Java robot for FRC Team YCS Robotics, 2026 season. The robot shoots fuel into a hub, intakes from the floor, and hangs. Vision localization drives an aim-and-shoot feature.

**Git tags:**
- `baseline-2026-cc` — original WCP Swerve CC import
- `season-2026-final` — code as it ran at competition (MISAL Q74, March 29 2026)
- `offseason-baseline-2026` — competition code with known localization bugs fixed; offseason starting point

---

## Library Constraints

This codebase is built on three primary libraries. All recommendations must stay consistent with their intended APIs and patterns — do not introduce custom alternatives that duplicate functionality these libraries already provide.

**CTRE Phoenix 6 / Tuner X**
- Swerve drivetrain, swerve modules, and the Pigeon2 gyro are configured and generated via **CTRE Tuner X**. `TunerConstants.java` is the generated output — do not hand-edit hardware IDs, gear ratios, or PID gains there; those belong in Tuner X and are regenerated.
- **After any Tuner X regeneration:** `driveInitialConfigs` should remain a plain `new TalonFXConfiguration()` — do not add current limits there. Drive motor supply current limit is applied in `Swerve.java` after framework initialization (adding it to `driveInitialConfigs` caused spin-on-enable). Hoot log path is set in `Robot.java` via `SignalLogger.setPath()` — `kCANBus` needs no path argument.
- Motor controllers (TalonFX) use Phoenix 6 APIs (`TalonFX`, `StatusSignal`, control requests). Do not mix in Phoenix 5 (`WPI_TalonFX`, `set()`, `.configXxx()`) patterns.
- Use CTRE's `SwerveRequest` types (`FieldCentric`, `FieldCentricFacingAngle`, `ApplyFieldSpeeds`, etc.) for all drivetrain control. Do not construct raw `ChassisSpeeds` and bypass the request layer.

**WPILib**
- Use the WPILib command-based framework (`Command`, `Subsystem`, `CommandScheduler`). Do not spawn threads or use manual periodic polling outside of subsystem `periodic()` methods.
- Pose estimation uses WPILib's `SwerveDrivePoseEstimator` (accessed through CTRE's `TunerSwerveDrivetrain`). Do not introduce a second estimator or shadow pose variable.
- Use `Rotation2d` for all angles so they stay bounded. Raw `double` degrees/radians should only appear at the boundary where hardware APIs require them.
- Autonomous paths use the **Choreo** library (`AutoFactory`, `.traj` files). Do not introduce PathPlanner or other path-following libraries.

**Limelight**
- Vision data is consumed via the `LimelightHelpers` utility class (HTTP/NetworkTables interface). Do not access NetworkTables vision keys manually — use the `LimelightHelpers` API.
- MegaTag2 is the primary XY source; MegaTag1 is used for rotation cross-check and pose initialization. Any new vision feature should respect this two-tag-type architecture.
- Always call `LimelightHelpers.SetRobotOrientation()` before reading MegaTag2 results each loop — MegaTag2's XY solve is only valid when the heading is current.

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
- **Fixed:** No field boundary validation — implemented in `Limelight.java` (out-of-field rejection + `kMaxVisionJumpMeters` gate).
- **Fixed:** No camera health/disconnect monitoring — implemented via `hasRecentMeasurement()` with `kCameraHealthTimeoutSeconds = 0.5`.
- **Open:** Odometry stddev in `Swerve` constructor (`VecBuilder.fill(0.1, 0.1, 0.1)`) has not been tuned relative to the new dynamic vision stddev. Tune last — depends on `kVisionXYStdDevCoefficient` being validated first.
- **Open:** `kVisionXYStdDevCoefficient = 0.02` is a starting point and needs empirical tuning against match logs.

### Known path tracking issues (from MISAL Q74 log analysis)
- **Fixed:** `kMaxVisionJumpMeters` lowered from 1.0 to 0.5m in `Constants.java` — a 0.36m mid-trajectory localization jump passed through the old gate in Q74.
- **Open:** Path feedback too aggressive — `kMaxPathFeedbackSpeedMps = 0.35` and X/Y PID P=10 in `Swerve.java` caused sustained 0.20–0.25 m/s overshoot above the 0.80 m/s trajectory max. Reduce clamp to 0.15 and P to 5; tune on practice field.
- **Open:** Deceleration lag cascades from overshoot above. Also verify Choreo robot model mass (62.14 kg) matches current build weight and regenerate trajectory if different.
