package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    private static final Translation2d kBlueHubPosition =
        new Translation2d(Inches.of(182.105), Inches.of(158.845));
    private static final Translation2d kRedHubPosition =
        new Translation2d(Inches.of(469.115), Inches.of(158.845));

    /** Blue hub (near origin); Red hub (far end). When no FMS, defaults to Blue. */
    public static Translation2d hubPosition() {
        final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Blue) {
            return kBlueHubPosition;
        }
        return kRedHubPosition;
    }

    /**
     * Known pose for practice: robot placed in front of the hub for the current alliance.
     * Used when FMS is not attached; place the robot at this position and the code will set
     * odometry to match. Offset from hub center (inches): positive = toward own alliance wall.
     */
    // Practice start: robot is offset from the hub *center* by this many inches.
    // (Hub face distance = this value minus half hub diameter; robot pose is measured at robot center.)
    private static final double kPracticeOffsetFromHubInches = 41.0;

    /** Full pose (x, y, heading) for practice start. Alliance from DS (FMS or manual). */
    public static Pose2d practiceInitialPose() {
        final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        final Translation2d hub = hubPosition();
        final double offsetMeters = Inches.of(kPracticeOffsetFromHubInches).in(Meters);
        final Translation2d position;
        final Rotation2d heading;
        if (alliance == Alliance.Blue) {
            position = hub.plus(new Translation2d(-offsetMeters, 0));
            heading = Rotation2d.kZero;
        } else {
            position = hub.plus(new Translation2d(offsetMeters, 0));
            heading = Rotation2d.k180deg;
        }
        return new Pose2d(position, heading);
    }

    /** Fixed practice pose: placed in front of the Red hub, heading 180° (alliance-independent). */
    public static Pose2d practiceRedHubPose() {
        final double offsetMeters = Inches.of(kPracticeOffsetFromHubInches).in(Meters);
        final Translation2d position = kRedHubPosition.plus(new Translation2d(offsetMeters, 0));
        return new Pose2d(position, Rotation2d.k180deg);
    }
}
