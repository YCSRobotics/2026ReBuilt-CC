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
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
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
