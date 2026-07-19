// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * When true, no mechanism hardware (CAN 50+) is created — swerve-only bringup, no CAN timeouts.
     * Set false when mechanisms are on the robot; then set each kFloor, kIntakePivot, etc. true as added.
     */
    public static final boolean kSwerveOnlyBringup = false;

    /**
     * Set true for each mechanism that is physically on the robot (CAN IDs 50+).
     * When false, that subsystem does not create hardware and all commands no-op (no CAN errors).
     * Ignored when {@link #kSwerveOnlyBringup} is true (all treated as absent).
     * If you see timeout messages for a mechanism that is false, they may be from REV Hardware Client
     * or other tools scanning the bus—our code does not create that device when false.
     */
    public static final class MechanismPresence {
        private MechanismPresence() {}

        public static boolean kFloor() { return !kSwerveOnlyBringup && kFloorPresent; }
        public static boolean kIntakePivot() { return !kSwerveOnlyBringup && kIntakePivotPresent; }
        public static boolean kIntakeRollers() { return !kSwerveOnlyBringup && kIntakeRollersPresent; }
        public static boolean kFeeder() { return !kSwerveOnlyBringup && kFeederPresent; }
        public static boolean kShooter() { return !kSwerveOnlyBringup && kShooterPresent; }
        public static boolean kHanger() { return !kSwerveOnlyBringup && kHangerPresent; }

        private static final boolean kFloorPresent = true;   // competition: manual shoot feed sequence
        private static final boolean kIntakePivotPresent = true;  // pivot bringup: ID 51 on CAN
        private static final boolean kIntakeRollersPresent = true;  // intake rollers on CAN (Ports.kIntakeRollers)
        private static final boolean kFeederPresent = true;   // competition: shooter/feeder sequence
        private static final boolean kShooterPresent = true;  // competition shooter bringup; aimAndShoot / shootManually
        private static final boolean kHangerPresent = false;
    }

    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    /** Feeder + shooter competition sequence (no hood/floor/intake). */
    public static final class ShooterFeeder {
        private ShooterFeeder() {}

        /** Seconds to run feeder after shooter is at speed (one note). Tune at field. */
        public static final double kFeedDurationSeconds = 0.35;
    }

    /** Feed sequence: shooter at speed → feeder on at t=0.05s → floor on 0.15s later (t=0.20s). */
    public static final class FeedSequence {
        private FeedSequence() {}

        /** Delay (s) from feed start until feeder and floor turn on (both together). */
        public static final double kFeederDelaySeconds = 0;
        /** Delay (s) from feed start until floor turns on (0 = same time as feeder). */
        public static final double kFloorDelaySeconds = 0;
    }

    /** REV Neo 2.0 (used on Feeder). Free speed ~5670 RPM at 12 V. */
    public static class Neo2 {
        public static final AngularVelocity kFreeSpeed = RPM.of(5670);
    }

    public static class Field {
        /** FRC 2026 field length (X axis, Blue wall to Red wall). Confirmed from field drawing: 651.22 in. */
        public static final double kFieldLengthMeters = Inches.of(651.22).in(Meters);
        /** FRC 2026 field width (Y axis). Confirmed from field drawing: 317.69 in. */
        public static final double kFieldWidthMeters = Inches.of(317.69).in(Meters);
    }

    public static class Limelight {
        // Camera pose relative to robot center (robot coordinate system)
        // Forward: positive = toward front bumper from robot center (meters)
        // Side: positive = right side of robot (meters)
        // Up: positive = upward from robot center (meters)
        // Roll, Pitch, Yaw: rotation angles in degrees
        
        public static final Distance kCameraForwardOffset = Meters.of(-0.275);  // Camera is 0.275 m toward back from robot center
        public static final Distance kCameraSideOffset = Meters.of(0.0);        // Centered left/right
        public static final Distance kCameraUpOffset = Meters.of(0.66);         // Height above robot center
        
        public static final double kCameraRollDegrees = 0.0;   // Rotation around forward axis
        public static final double kCameraPitchDegrees = 18;   // Rotation around side axis (positive = camera pointing up)
        public static final double kCameraYawDegrees = 0.0;    // Rotation around vertical axis (positive = camera pointing right)

        /**
         * If |vision yaw − odometry yaw| is within this band (degrees), fuse vision rotation into pose;
         * otherwise use vision translation only (measurement pose keeps odometry heading, theta std dev huge).
         */
        public static final double kVisionHeadingAgreementDegrees = 7.5;

        /**
         * Coefficient for dynamic XY std dev scaling: xyStdDev = coefficient * dist² / tagCount²
         * Increase to trust vision less; decrease to trust it more. Needs empirical tuning.
         * Reference: 6328 Mechanical Advantage uses 0.01 with 4 cameras + custom hardware.
         */
        public static final double kVisionXYStdDevCoefficient = 0.02;

        /**
         * Minimum XY std dev floor (meters) — prevents overconfidence at very close range.
         */
        public static final double kVisionXYStdDevMinMeters = 0.05;

        /**
         * Theta std dev (radians) when heading is within {@link #kVisionHeadingAgreementDegrees}
         * and MegaTag1 saw only 1 tag. Single-tag rotation is ambiguity-prone; trust moderately.
         */
        public static final double kVisionThetaStdDevSingleTagRad = 0.35;

        /**
         * Theta std dev (radians) when heading is within {@link #kVisionHeadingAgreementDegrees}
         * and MegaTag1 saw 2+ tags. Multi-tag rotation is more reliable; trust more.
         */
        public static final double kVisionThetaStdDevMultiTagRad = 0.15;

        /**
         * Theta std dev (radians) when outside the heading band: effectively translation-only fusion.
         */
        public static final double kVisionThetaStdDevTranslationOnlyRad = 1.0e6;

        /**
         * Minimum XY std dev floor (meters) when outside the heading band. Higher than the normal
         * floor because MegaTag2 XY quality is correlated with heading quality — if heading is
         * suspect enough to be outside the band, trust XY less too.
         */
        public static final double kVisionXYStdDevOutOfBandMinMeters = 0.3;

        /**
         * Maximum allowed distance (meters) between a new MegaTag2 XY estimate and the current
         * odometry pose. Measurements beyond this are rejected as implausible solves.
         * At max robot speed (0.8 m/s) with 23 ms vision latency, legitimate odometry drift
         * is ~18 mm. 0.5 m catches bad solves (a 0.36 m jump was observed in Q74) while
         * still allowing real corrections after brief vision outages.
         */
        public static final double kMaxVisionJumpMeters = 0.5;

        /**
         * Camera health timeout (seconds). If no valid measurement has been accepted within this
         * window, the vision health indicator on SmartDashboard turns false.
         */
        public static final double kCameraHealthTimeoutSeconds = 0.5;
    }
}
