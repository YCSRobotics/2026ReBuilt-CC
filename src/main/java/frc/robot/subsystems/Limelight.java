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
import edu.wpi.first.wpilibj.Timer;
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
    private double m_lastValidMeasurementTimestamp = -1.0;
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private final NetworkTableEntry distToTagRobotM;
    private final NetworkTableEntry distToTagCameraM;
    private final NetworkTableEntry distToTagRobotIn;
    private final NetworkTableEntry aprilTagCountEntry;
    private final NetworkTableEntry visionOdometryYawDeltaDegEntry;
    private final NetworkTableEntry visionHeadingWithinBandEntry;
    private final NetworkTableEntry visionPoseOutOfFieldEntry;
    private final NetworkTableEntry visionCameraHealthyEntry;

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
        this.visionPoseOutOfFieldEntry = telemetryTable.getEntry("Vision/Pose Out Of Field");
        this.visionCameraHealthyEntry = telemetryTable.getEntry("Vision/Camera Healthy");

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
        // Hub used for aim/distance (Landmarks); publish so DS / AdvantageScope can verify Red vs Blue selection.
        final Translation2d hubField = Landmarks.hubPosition();
        SmartDashboard.putNumber("Hub X (m)", hubField.getX());
        SmartDashboard.putNumber("Hub Y (m)", hubField.getY());

        visionCameraHealthyEntry.setBoolean(hasRecentMeasurement());

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
                    final double ix = mt1.pose.getX();
                    final double iy = mt1.pose.getY();
                    final boolean initOnField = ix >= 0 && ix <= Constants.Field.kFieldLengthMeters
                        && iy >= 0 && iy <= Constants.Field.kFieldWidthMeters;
                    if (initOnField) {
                        swerve.resetPoseAndGyro(mt1.pose);
                        m_hasInitializedPose = true;
                    }
                }
            }

            // Always run vision, but reject any measurement captured before the last pose reset.
            // This prevents stale 0° measurements (from before teleop/auto pose init) from
            // overwriting the correct heading after resetPoseAndGyro() is called.
            final double yawDeg = currentRobotPose.getRotation().getDegrees();
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

    /**
     * Returns true if a valid vision measurement was accepted within the camera health timeout.
     * Use as a driver dashboard indicator — not a hard fire gate (odometry remains valid short-term).
     */
    public boolean hasRecentMeasurement() {
        return m_lastValidMeasurementTimestamp >= 0
            && (Timer.getFPGATimestamp() - m_lastValidMeasurementTimestamp)
                <= Constants.Limelight.kCameraHealthTimeoutSeconds;
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

        // Reject if any visible tag has high ambiguity — the PnP solver cannot reliably distinguish
        // between the two candidate poses, so accepting the measurement risks injecting a bad solve.
        // This is the root-cause fix for large mid-trajectory jumps (e.g. MISAL Q74).
        // MegaTag2 is largely immune (heading constraint eliminates the mirror solution), but the
        // fiducials come from the same frame, so this check covers both estimates together.
        final RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        if (fiducials != null) {
            for (RawFiducial tag : fiducials) {
                if (tag.ambiguity > Constants.Limelight.kMaxTagAmbiguity) {
                    return Optional.empty();
                }
            }
        }

        // Combine MegaTag2 translation with MegaTag1 rotation as the raw vision estimate.
        final Rotation2d visionRotation = poseEstimate_MegaTag1.pose.getRotation();
        final var translation = poseEstimate_MegaTag2.pose.getTranslation();
        final Rotation2d odometryRotation = currentRobotPose.getRotation();
        final double yawDeltaDeg = Math.abs(visionRotation.minus(odometryRotation).getDegrees());
        visionOdometryYawDeltaDegEntry.setDouble(yawDeltaDeg);

        final boolean withinHeadingBand = yawDeltaDeg <= Constants.Limelight.kVisionHeadingAgreementDegrees;
        visionHeadingWithinBandEntry.setBoolean(withinHeadingBand);

        // Dynamic XY std dev: trust scales with distance² and inversely with tagCount².
        // A single tag at 5m gets ~0.50m stddev; two tags at 2m get ~0.05m stddev.
        // The floor applied depends on heading band agreement — outside the band, MegaTag2 XY
        // quality is less certain because it depends on heading quality, so a higher floor is used.
        final double xyStdDevRaw = Constants.Limelight.kVisionXYStdDevCoefficient
            * Math.pow(poseEstimate_MegaTag2.avgTagDist, 2.0)
            / Math.pow(poseEstimate_MegaTag2.tagCount, 2.0);

        final Pose2d fusedPose;
        final Matrix<N3, N1> standardDeviations;
        if (withinHeadingBand) {
            // Vision and odometry agree on heading: allow Kalman to nudge theta toward vision.
            // Multi-tag MegaTag1 rotation is more reliable than single-tag; trust it more.
            final double thetaStdDev = poseEstimate_MegaTag1.tagCount >= 2
                ? Constants.Limelight.kVisionThetaStdDevMultiTagRad
                : Constants.Limelight.kVisionThetaStdDevSingleTagRad;
            final double xyStdDev = Math.max(Constants.Limelight.kVisionXYStdDevMinMeters, xyStdDevRaw);
            fusedPose = new Pose2d(translation, visionRotation);
            standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
        } else {
            // Disagree: translation only — measurement uses odometry yaw so estimator does not snap heading.
            // Higher XY floor because MegaTag2 XY quality correlates with heading quality.
            final double xyStdDev = Math.max(Constants.Limelight.kVisionXYStdDevOutOfBandMinMeters, xyStdDevRaw);
            fusedPose = new Pose2d(translation, odometryRotation);
            standardDeviations = VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                Constants.Limelight.kVisionThetaStdDevTranslationOnlyRad
            );
        }

        poseEstimate_MegaTag2.pose = fusedPose;

        // Reject measurements that place the robot outside the field boundary.
        // A bad MegaTag solve can produce wildly off-field poses; injecting them corrupts the Kalman filter.
        final double x = fusedPose.getX();
        final double y = fusedPose.getY();
        final boolean outOfField = x < 0 || x > Constants.Field.kFieldLengthMeters
            || y < 0 || y > Constants.Field.kFieldWidthMeters;
        visionPoseOutOfFieldEntry.setBoolean(outOfField);
        if (outOfField) {
            return Optional.empty();
        }

        posePublisher.set(poseEstimate_MegaTag2.pose);
        m_lastValidMeasurementTimestamp = Timer.getFPGATimestamp();

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
