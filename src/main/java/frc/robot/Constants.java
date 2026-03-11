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
    /** When true, only drive + operator bringup test buttons are active (one mechanism per button). Set false for competition. */
    public static final boolean kBringupMode = false;

    /**
     * Constants used only when {@link #kBringupMode} is true (subsystem bringup / test).
     * Search for "Bringup" in the codebase to find all bringup-specific logic.
     */
    public static final class Bringup {
        private Bringup() {}

        /**
         * Shooter: velocity setpoint (RPM) for Operator Y hold. Uses closed-loop for soft spool; not raw percent.
         * Expected bringup behavior: Hold Operator Y → shooter spins at this RPM (velocity PID); release → stop (coast).
         */
        public static final double kShooterRPM = 800.0;
    }

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

    /** REV Neo 2.0 (used on Feeder). Free speed ~5670 RPM at 12 V. */
    public static class Neo2 {
        public static final AngularVelocity kFreeSpeed = RPM.of(5670);
    }

    public static class Limelight {
        // Camera pose relative to robot center (robot coordinate system)
        // Forward: positive = forward from robot center (meters)
        // Side: positive = right side of robot (meters)
        // Up: positive = upward from robot center (meters)
        // Roll, Pitch, Yaw: rotation angles in degrees
        
        public static final Distance kCameraForwardOffset = Meters.of(0.275);  // Forward from robot center
        public static final Distance kCameraSideOffset = Meters.of(0.335);     // Right side (positive) or left side (negative)
        public static final Distance kCameraUpOffset = Meters.of(0.48);        // Height above robot center
        
        public static final double kCameraRollDegrees = 0.0;   // Rotation around forward axis
        public static final double kCameraPitchDegrees = 0.0;  // Rotation around side axis (positive = camera pointing up)
        public static final double kCameraYawDegrees = 0.0;    // Rotation around vertical axis (positive = camera pointing right)
    }
}
