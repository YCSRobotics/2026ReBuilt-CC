package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.Optional;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kMaxPathFeedbackSpeedMps = 0.35;
    private static final double kMaxPathFeedbackOmegaRadPerSec = 0.9;
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private Alliance m_lastOperatorPerspectiveAlliance = null;
    private double m_lastPoseResetTimestamp = 0.0;

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds pathFieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);

    public Swerve() {
        super(
            TunerConstants.DrivetrainConstants, 
            0,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.1, 0.1, 0.1),
            TunerConstants.FrontLeft, 
            TunerConstants.FrontRight, 
            TunerConstants.BackLeft, 
            TunerConstants.BackRight
        );
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPoseAndGyro,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }

    /**
     * Resets the robot pose and heading in the drivetrain estimator.
     * Uses resetPose() only — calling resetRotation() alongside it causes a double-offset
     * bug where the heading doubles to 0° for non-zero headings (e.g. Red alliance 180°).
     */
    public void resetPoseAndGyro(Pose2d pose) {
        m_lastPoseResetTimestamp = Utils.getCurrentTimeSeconds();
        resetPose(pose);
    }

    /** Returns the timestamp (seconds) of the most recent pose reset. Used by Limelight to reject stale measurements. */
    public double getLastPoseResetTimestamp() {
        return m_lastPoseResetTimestamp;
    }

    /**
     * Sets the current pose heading to the alliance "forward" direction (0° Blue, 180° Red)
     * while keeping the current position. Use when the robot is physically facing the
     * opposing alliance wall so field-centric and aim-to-shoot use the correct reference.
     */
    public void resetHeadingToAllianceForward() {
        final Pose2d current = getState().Pose;
        final Translation2d position = current.getTranslation();
        final Rotation2d allianceForward = DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red
            ? kRedAlliancePerspectiveRotation
            : kBlueAlliancePerspectiveRotation;
        resetPose(new Pose2d(position, allianceForward));
        resetRotation(allianceForward);
    }

    /**
     * Forces operator perspective forward to match the fixed practice pose (Red hub, heading 180°).
     * This avoids field-centric "fight" when practicing with no FMS alliance information.
     */
    public void setOperatorPerspectiveForPracticeRed() {
        setOperatorPerspectiveForward(kRedAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
        m_lastOperatorPerspectiveAlliance = Alliance.Red;
    }

    /**
     * True when chassis translational and rotational speed are below a small threshold.
     * Used to ensure the robot is stopped before shooting for consistency.
     */
    public boolean isStopped() {
        final ChassisSpeeds s = getState().Speeds;
        final double linearToleranceMps = 0.05;
        final double omegaToleranceRadPerSec = 0.1;
        return Math.abs(s.vxMetersPerSecond) < linearToleranceMps
            && Math.abs(s.vyMetersPerSecond) < linearToleranceMps
            && Math.abs(s.omegaRadiansPerSecond) < omegaToleranceRadPerSec;
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        final double xFeedbackMps = MathUtil.clamp(
            pathXController.calculate(pose.getX(), sample.x),
            -kMaxPathFeedbackSpeedMps,
            kMaxPathFeedbackSpeedMps
        );
        final double yFeedbackMps = MathUtil.clamp(
            pathYController.calculate(pose.getY(), sample.y),
            -kMaxPathFeedbackSpeedMps,
            kMaxPathFeedbackSpeedMps
        );
        final double thetaFeedbackRadPerSec = MathUtil.clamp(
            pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading),
            -kMaxPathFeedbackOmegaRadPerSec,
            kMaxPathFeedbackOmegaRadPerSec
        );

        targetSpeeds.vxMetersPerSecond += xFeedbackMps;
        targetSpeeds.vyMetersPerSecond += yFeedbackMps;
        targetSpeeds.omegaRadiansPerSecond += thetaFeedbackRadPerSec;

        setControl(
            pathFieldSpeedsRequest.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        final Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        final boolean allianceKnown = allianceOpt.isPresent();

        // Apply operator perspective when:
        // 1) We haven't applied it before (initial startup)
        // 2) DS is disabled AND alliance is known AND alliance changed
        if (!m_hasAppliedOperatorPerspective
            || (DriverStation.isDisabled() && allianceKnown && allianceOpt.get() != m_lastOperatorPerspectiveAlliance)) {

            final Alliance allianceToApply = allianceOpt.orElse(Alliance.Blue);
            setOperatorPerspectiveForward(
                allianceToApply == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
            m_lastOperatorPerspectiveAlliance = allianceToApply;
        }

        /* Publish raw Pigeon 2 yaw for heading troubleshooting (SmartDashboard is on NT) */
        SmartDashboard.putNumber("Pigeon Raw Yaw (deg)", getPigeon2().getYaw().getValueAsDouble());
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
