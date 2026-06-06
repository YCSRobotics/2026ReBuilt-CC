# MISAL Q74 Localization Analysis
**Match:** MISAL Q74 — March 29, 2026
**Log file:** FRC_20260329_160135_MISAL_Q74.csv
**Analyzed:** April 2026

---

## Match Summary

| Phase | Timestamp | Rows |
|-------|-----------|------|
| Auto | T=110.6s to T=125.1s | 726 rows |
| Disabled gap | T=125.1s to T=138.9s | — |
| Teleop | T=138.9s to T=275.1s | 6,809 rows |

---

## Key Metrics from Log

| Signal | Auto | Teleop |
|--------|------|--------|
| DriveState Pose X range | 0.0 – 14.1 m | -0.2 – 15.7 m |
| DriveState Pose Y range | 0.0 – 4.1 m | -3.6 – 7.4 m |
| Pose jumps > 0.5 m | 0 | **146** |
| Pose jumps > 3.0 m | 0 | **28** |
| Vision yaw delta mean | — | 1.1° |
| Vision yaw delta std dev | — | 7.7° |
| Vision yaw delta max abs | — | **107.0°** |
| Vision Heading Within Band = False | — | 32% of frames (2,206 rows) |
| robot_orientation_set/0 range | — | **-68.1° to 1819.8°** |
| AprilTag count (avg when visible) | — | 1.6 tags |
| tv=1 (target valid) | — | 70% of frames |
| Pipeline latency (tl) avg | — | 11.8 ms |
| Capture latency (cl) avg | — | 11.4 ms |
| Total vision latency avg | — | **23.3 ms** (max 39.6 ms) |

---

## Root Cause Findings

### 1. CRITICAL — Yaw sent to Limelight was unbounded
**Signal:** `NT:/limelight/robot_orientation_set/0`
**Observed:** Range of **-68.1° to 1819.8°** during teleop
**Expected:** Should stay within ±180°

The robot was feeding the raw accumulated Pigeon2 gyro angle to
`LimelightHelpers.SetRobotOrientation()` without normalizing it.
Limelight MegaTag2 uses this heading to resolve AprilTag pose ambiguity.
An unbounded yaw causes the Limelight to compute a completely wrong
field-relative robot pose, which then gets injected into
`SwerveDrivePoseEstimator` as a large spurious correction.

**Root cause of the unbounded value:** `getPigeon2().getYaw()` in
CTRE Phoenix 6 is a continuously-accumulating hardware register — it
never wraps. Every rotation adds to it permanently. After 5 full
rotations from a starting point of 0°, it reads 1800°, not 0°.

**Why it worked in auto:** `resetPoseAndGyro()` calls CTRE's
`resetPose()` which calls `pigeon2.setYaw()`, resetting the raw
register at match start. During the short auto period, rotation was
limited and the value stayed near ±180°. In a full 2-minute teleop
with free driving, the register walked to 1820°.

**Why the fix was lost:** The correct fix (`currentRobotPose.getRotation().getDegrees()`)
was applied in commit `d2621aa` on March 28. The `IntakePivotAgitate`
branch was not rebased after this fix landed. When PR #7
(`IntakePivotAgitate`) was merged on match morning (March 29), it
reverted `d2621aa` because its version of `Limelight.java` was based
on the pre-fix code.

**Fix (commit `26a0897`):** Use `currentRobotPose.getRotation().getDegrees()`
instead of `getPigeon2().getYaw().getValueAsDouble()`. `Rotation2d`
stores heading as sin/cos internally and `getDegrees()` uses
`Math.atan2()`, which always returns ±180°.

---

### 2. HIGH — Vision XY stddev was flat regardless of tag quality
**Signal:** `NT:/SmartDashboard/limelight/AprilTag Count`,
`NT:/SmartDashboard/limelight/Distance to Tag - Robot (m)`
**Observed:** Code used a flat `kVisionStdDevXYMeters = 0.1` for all frames.
Average tags visible: 1.6. Many single-tag frames at varying distances.

A flat 0.1m stddev treats a single tag at 5m identically to three
tags at 1m. The Kalman filter was trusting distant single-tag estimates
at the same weight as close multi-tag estimates, causing unnecessary
pose corrections from noisy measurements.

**Fix (commit `19b0466`):** Replace flat constant with a continuous
formula adapted from Team 6328 Mechanical Advantage:

```
xyStdDev = max(0.05, 0.02 × avgTagDist² / tagCount²)
```

This makes stddev scale naturally — distant single-tag estimates
get high stddev (low trust) and close multi-tag estimates get low
stddev (high trust). The coefficient 0.02 is a starting point and
needs empirical tuning.

---

### 3. OPEN — Vision yaw delta rejection not hard enough
**Signal:** `NT:/SmartDashboard/limelight/Vision Heading Within Band`
**Observed:** 32% of teleop frames outside the 7.5° heading band.
Max yaw delta: 107°.

The code has a `withinHeadingBand` check that falls back to
translation-only mode outside the band, but still injects MegaTag2
XY translation in both cases. With Fix 1 applied (correct heading
to MegaTag2), the 32% out-of-band rate should reduce significantly
as MegaTag2 XY becomes reliable. Remaining out-of-band cases are
caused by MegaTag1 single-tag pose ambiguity (not heading error).

**Status:** Re-evaluate after observing with Fix 1 in place.

---

## What Was Working Well

| Signal | Status | Notes |
|--------|--------|-------|
| Auto localization | ✓ Good | Zero jumps, pose tracked correctly 0–14m |
| Vision latency compensation | ✓ Good | 23ms avg, well within addVisionMeasurement timestamp handling |
| Odometry (wheel + gyro) | ✓ Good | Pose stable whenever vision injection was absent |
| Multi-tag detection | ✓ Good | 2+ tags visible 44% of teleop — leverage more aggressively |

---

## Raw Log File

The raw CSV log (`FRC_20260329_160135_MISAL_Q74.csv`) is stored in
the YCS Robotics shared Google Drive under:
`2026 Season / Match Logs / FRC_20260329_160135_MISAL_Q74.csv`

---

## Open Items

See [CLAUDE.md](../../CLAUDE.md) for the current list of open localization
issues and offseason goals.
