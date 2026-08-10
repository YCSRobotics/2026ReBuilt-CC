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

## Competition Day Checklist

**Before first match of the day — RoboRIO disk space**
Phoenix SignalLogger (hoot files) fills RoboRIO storage silently. When the disk is full it stops writing with no DS warning — you lose all log data for subsequent matches. This happened at MIANN Q16 (2026-08-01).

```bash
# Check available space (connect via USB or robot WiFi)
ssh lvuser@10.TEAM.2 "df -h /home/lvuser && ls -lh /home/lvuser/logs/ | tail -20"
```

- Copy any hoot files you want to keep to the DS laptop first
- Then delete all hoot files before the competition starts:
  ```bash
  ssh lvuser@10.TEAM.2 "rm /home/lvuser/logs/*.hoot"
  ```
- RoboRIO has ~512MB–1GB usable. A full match day (8+ matches + practice) fills it easily with a CANivore logging at high frequency.

**Between practice matches — pose initialization without power cycle**

At the end of each practice match, **driver steers the robot back to the hub starting position before disabling.**

Why: `m_hasInitializedPose` is never reset on disable (intentional). With the hard jump gate removed, Kalman fusion corrects any pose error within 1–2 seconds of driving once tags are visible. But driving to hub under power at match-end means the next match starts with an already-correct pose — no correction needed. Powered driving preserves accurate odometry; pushing the robot after disable does not.

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

**Pose initialization:** On boot, `getMeasurement()` uses MegaTag1 (not MegaTag2) to initialize the pose, requiring 2+ tags. If vision never initializes (e.g. robot touching hub — tags fill FOV, no valid solve), teleop start falls back to `Landmarks.practiceInitialPose()`, which computes the hub-front position geometrically per alliance. Vision self-corrects within ~1–2 seconds of driving away from the hub once tags are visible. `m_hasInitializedPose` is intentionally never reset on disable — resetting would snap the pose to hub-front if the robot brownouts and re-enables mid-match.

**Do not use hard distance gates for vision rejection.** A hard threshold (e.g. reject if jump > 0.5m) cannot distinguish between a bad MegaTag solve (should reject) and a legitimate vision correction of accumulated odometry drift (should accept). After 10+ seconds of aggressive driving, drift can exceed 0.5m — a gate rejects all corrections and creates a positive feedback loop: drift → rejection → more drift → pose increasingly wrong. The Kalman filter's dynamic std devs are the correct mechanism: a measurement far from current pose gets high std devs (low trust) and is blended in slowly, not hard-rejected. Use `kMaxTagAmbiguity` (ambiguity score) to reject genuinely bad solves at the source instead.

### Aim-and-shoot — `AimAndDriveCommand` + `PrepareShotCommand` + `SubsystemCommands`
`AimAndDriveCommand` uses CTRE's `FieldCentricFacingAngle` request in **OperatorPerspective** — the same perspective as `ManualDriveCommand` — so driver stick feel is consistent on both alliances. CTRE handles the alliance flip internally. It rotates the robot to face `Landmarks.hubPosition()`, which returns the correct hub (blue or red) based on the Driver Station alliance. `isAimed()` and `getDirectionToHub()` apply `.rotateBy(swerve.getOperatorForwardDirection())` to keep all heading math in the same frame as the request.

`PrepareShotCommand` has **two distinct shot constant sets** that serve different purposes — do not merge them:
- **Interpolation table** (`*_ROW_SHOT`): anchors for the `InterpolatingTreeMap` used by dynamic `aimAndShoot()`. Distance is measured center-to-center at runtime from the pose estimator.
- **Button presets** (`*_KEY_SHOT`): fixed RPM/hood values tied to operator buttons (Y/X/B/A) and autonomous. Bypass interpolation entirely — driver drives to a known position and presses the button. Values were field-tuned independently and intentionally differ from the interpolation anchors.

`SubsystemCommands` is a **command factory, not a subsystem** — it holds no hardware, just composes commands from subsystems passed in at construction. `aimAndShoot()` fires when aimed + shooter at speed + translation stopped (not full stop — checking omega would fight the heading PID).

### Autonomous — `AutoRoutines.java` + Choreo trajectories
Trajectories live in `src/main/deploy/choreo/` as `.traj` files. They are loaded by name at runtime (no code generation required). Multi-segment trajectories use integer segment indices (0, 1, 2…). The auto chooser is published to SmartDashboard.

During Choreo segments, the Limelight default command is preempted by the trajectory follower. Any segment that should keep vision running must explicitly call `limelight.idle(swerve)` with `.whileTrue()`.

### Alliance handling — `Landmarks.java`
`Landmarks.hubPosition()` and `Landmarks.hubPosition()` are alliance-aware and read from the Driver Station at call time. When FMS is not attached, both default to Blue. Use `Landmarks.practiceRedHubPose()` for a fixed Red-side practice placement that bypasses the DS alliance check.

---

## Working with this Codebase

**Check the baseline git tag before proposing fixes for regressions.**
If a bug looks like something that used to work but broke when someone changed it, diff against `baseline-2026-cc` first:
```bash
git show baseline-2026-cc:src/main/java/frc/robot/commands/SomeCommand.java
git diff baseline-2026-cc HEAD -- src/main/java/frc/robot/commands/SomeCommand.java
```
The original correct design is often already there. Someone removed it without understanding the full consequences. The `AimAndDriveCommand` OperatorPerspective fix (2026-08-05) is the canonical example — the baseline had the complete correct implementation; the "fix" needed was restoring it, not designing something new.

**Research the full codebase before drawing conclusions.**
Before stating that telemetry, wiring, or functionality is "missing," grep for related classes and registration calls across the full `src/` tree. Reading a single file is not sufficient for architectural questions. Example: `SwerveTelemetry.java` exists and is wired in `RobotContainer.java` via `registerTelemetry()` — reading only `Swerve.java` would incorrectly conclude telemetry is absent.

---

## Offseason Goals (through September 2026)

1. **Understand the baseline** — develop deep familiarity with every subsystem and command
2. **Localization and aim-and-shoot improvements** — the `offseason-baseline-2026` tag marks the starting point; remaining known issues are tracked as GitHub Issues
3. **Student engagement** — build a framework for high school students with limited coding experience to understand and contribute to the code using Claude Code

### Known localization issues (from MISAL Q74 log analysis)
Full analysis: [docs/analysis/MISAL_Q74_localization_analysis.md](docs/analysis/MISAL_Q74_localization_analysis.md)
- **Fixed:** Unbounded Pigeon2 yaw fed to `SetRobotOrientation` (raw value reached 1820°). Fixed in `26a0897`.
- **Fixed:** Flat XY stddev (0.1m) trusted distant single-tag estimates equally to close multi-tag estimates. Fixed in `19b0466`.
- **Fixed:** No field boundary validation — implemented in `Limelight.java` (out-of-field rejection).
- **Fixed:** Hard vision jump gate (`kMaxVisionJumpMeters`) removed entirely (2026-08-05) — it blocked legitimate drift corrections during teleop and created a drift feedback loop. Kalman filter std dev scaling is the correct rejection mechanism.
- **Fixed:** Fallback pose was hardcoded eyeballed values (Blue: x=3.0m — 0.58m off). Now uses `Landmarks.practiceInitialPose()` which computes hub-front geometrically per alliance.
- **Fixed:** No ambiguity gate — bad PnP solves (high ambiguity) could inject wrong pose. Now rejected via `kMaxTagAmbiguity = 0.7` in `getMeasurement()`.
- **Fixed:** No camera health/disconnect monitoring — implemented via `hasRecentMeasurement()` with `kCameraHealthTimeoutSeconds = 0.5`.
- **Open:** Odometry stddev in `Swerve` constructor (`VecBuilder.fill(0.1, 0.1, 0.1)`) has not been tuned relative to the new dynamic vision stddev. Tune last — depends on `kVisionXYStdDevCoefficient` being validated first.
- **Open:** `kVisionXYStdDevCoefficient = 0.02` is a starting point and needs empirical tuning against match logs.

### Known path tracking issues (from MISAL Q74 log analysis)
- **Fixed:** Hard vision jump gate removed entirely (2026-08-05); Q74 bad solve now addressed at source via `kMaxTagAmbiguity = 0.7` ambiguity rejection.
- **Open:** Path feedback too aggressive — `kMaxPathFeedbackSpeedMps = 0.35` and X/Y PID P=10 in `Swerve.java` caused sustained 0.20–0.25 m/s overshoot above the 0.80 m/s trajectory max. Reduce clamp to 0.15 and P to 5; tune on practice field.
- **Open:** Deceleration lag cascades from overshoot above. Also verify Choreo robot model mass (62.14 kg) matches current build weight and regenerate trajectory if different.
