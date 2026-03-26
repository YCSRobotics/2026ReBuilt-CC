package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {
    private static final double kNoTargetSentinel = -1.0;

    private boolean m_hasInitializedPose = false;
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private final NetworkTableEntry distToTagRobotM;
    private final NetworkTableEntry distToTagCameraM;
    private final NetworkTableEntry distToTagRobotIn;
    private final NetworkTableEntry aprilTagCountEntry;
    private final NetworkTableEntry visionOdometryYawDeltaDegEntry;
    private final NetworkTableEntry visionHeadingWithinBandEntry;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
        this.distToTagRobotM = telemetryTable.getEntry("Distance to Tag - Robot (m)");
        this.distToTagCameraM = telemetryTable.getEntry("Distance to Tag - Camera (m)");
        this.distToTagRobotIn = telemetryTable.getEntry("Distance to Tag - Robot (inches)");
        this.aprilTagCountEntry = telemetryTable.getEntry("AprilTag Count");
        this.visionOdometryYawDeltaDegEntry = telemetryTable.getEntry("Vision/Odom Yaw Delta (deg)");
        this.visionHeadingWithinBandEntry = telemetryTable.getEntry("Vision Heading Within Band");

        // Configure camera pose relative to robot center
        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            Constants.Limelight.kCameraForwardOffset.in(Meters),
            Constants.Limelight.kCameraSideOffset.in(Meters),
            Constants.Limelight.kCameraUpOffset.in(Meters),
            Constants.Limelight.kCameraRollDegrees,
            Constants.Limelight.kCameraPitchDegrees,
            Constants.Limelight.kCameraYawDegrees
        );
    }

    @Override
    public void periodic() {
        // Publish vision-based distance to AprilTag for dashboard and Elastic (always publish so keys exist in NT)
        final RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        if (fiducials != null && fiducials.length > 0) {
            aprilTagCountEntry.setDouble(fiducials.length);
            RawFiducial closest = fiducials[0];
            for (int i = 1; i < fiducials.length; i++) {
                if (fiducials[i].distToRobot < closest.distToRobot) {
                    closest = fiducials[i];
                }
            }
            distToTagRobotM.setDouble(closest.distToRobot);
            distToTagCameraM.setDouble(closest.distToCamera);
            distToTagRobotIn.setDouble(Meters.of(closest.distToRobot).in(Inches));
        } else {
            aprilTagCountEntry.setDouble(0);
            distToTagRobotM.setDouble(kNoTargetSentinel);
            distToTagCameraM.setDouble(kNoTargetSentinel);
            distToTagRobotIn.setDouble(kNoTargetSentinel);
        }
    }

    public String getName() {
        return name;
    }

    /**
     * Runs MegaTag2 fusion + dashboard telemetry. Use as the default command, or during auto
     * ({@link #idle(Swerve)}) so vision keeps updating while another command holds this subsystem.
     */
    public Command visionUpdateCommand(Swerve swerve) {
        return run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            SmartDashboard.putString(
                "Alliance",
                DriverStation.getAlliance().map(a -> a.name()).orElse("Unknown"));
            final Translation2d robotPosition = currentRobotPose.getTranslation();
            final Translation2d hubPosition = Landmarks.hubPosition();
            final Distance distanceToHub = Meters.of(robotPosition.getDistance(hubPosition));
            SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));

            if (!m_hasInitializedPose) {
                final PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
                if (mt1 != null && mt1.tagCount >= 2) {
                    swerve.resetPoseAndGyro(mt1.pose);
                    m_hasInitializedPose = true;
                }
            }

            // Always run vision, but reject any measurement captured before the last pose reset.
            // This prevents stale 0° measurements (from before teleop/auto pose init) from
            // overwriting the correct heading after resetPoseAndGyro() is called.
            final double yawDeg = swerve.getPigeon2().getYaw().getValueAsDouble();
            final double yawRateDegPerSec = Math.toDegrees(swerve.getState().Speeds.omegaRadiansPerSecond);
            final Optional<Measurement> measurement = getMeasurement(currentRobotPose, yawDeg, yawRateDegPerSec);
            measurement.ifPresent(m -> {
                if (m.poseEstimate.timestampSeconds > swerve.getLastPoseResetTimestamp()) {
                    swerve.addVisionMeasurement(
                        m.poseEstimate.pose,
                        m.poseEstimate.timestampSeconds,
                        m.standardDeviations);
                }
            });
        })
            .ignoringDisable(true);
    }

    /**
     * Same as {@link #visionUpdateCommand(Swerve)} — scheduled during Choreo segments so odometry
     * still gets vision corrections while the default command is preempted.
     */
    public Command idle(Swerve swerve) {
        return visionUpdateCommand(swerve);
    }

    /** Returns true if vision has already initialized the robot pose from AprilTags. */
    public boolean hasInitializedPose() {
        return m_hasInitializedPose;
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose, double yawDeg, double yawRateDegPerSec) {
        // MegaTag2 uses yaw/yawRate as IMU inputs for localization fusion.
        LimelightHelpers.SetRobotOrientation(
            name,
            yawDeg,
            yawRateDegPerSec,
            0, 0,
            0, 0
        );

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
            poseEstimate_MegaTag1 == null
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            visionOdometryYawDeltaDegEntry.setDouble(-1.0);
            visionHeadingWithinBandEntry.setBoolean(false);
            return Optional.empty();
        }

        // Combine MegaTag2 translation with MegaTag1 rotation as the raw vision estimate.
        final Rotation2d visionRotation = poseEstimate_MegaTag1.pose.getRotation();
        final var translation = poseEstimate_MegaTag2.pose.getTranslation();
        final Rotation2d odometryRotation = currentRobotPose.getRotation();
        final double yawDeltaDeg = Math.abs(visionRotation.minus(odometryRotation).getDegrees());
        visionOdometryYawDeltaDegEntry.setDouble(yawDeltaDeg);

        final boolean withinHeadingBand = yawDeltaDeg <= Constants.Limelight.kVisionHeadingAgreementDegrees;
        visionHeadingWithinBandEntry.setBoolean(withinHeadingBand);

        final Pose2d fusedPose;
        final Matrix<N3, N1> standardDeviations;
        if (withinHeadingBand) {
            // Vision and odometry agree on heading: allow Kalman to nudge theta toward vision.
            fusedPose = new Pose2d(translation, visionRotation);
            standardDeviations = VecBuilder.fill(
                Constants.Limelight.kVisionStdDevXYMeters,
                Constants.Limelight.kVisionStdDevXYMeters,
                Constants.Limelight.kVisionThetaStdDevWithinHeadingBandRad
            );
        } else {
            // Disagree: translation only — measurement uses odometry yaw so estimator does not snap heading.
            fusedPose = new Pose2d(translation, odometryRotation);
            standardDeviations = VecBuilder.fill(
                Constants.Limelight.kVisionStdDevXYMeters,
                Constants.Limelight.kVisionStdDevXYMeters,
                Constants.Limelight.kVisionThetaStdDevTranslationOnlyRad
            );
        }

        poseEstimate_MegaTag2.pose = fusedPose;

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}
