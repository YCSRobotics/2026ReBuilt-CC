// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.Landmarks;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
// import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private static final Pose2d kBackupAndShootExpectedStartPose = new Pose2d(
        13.136984825134276,
        3.964722871780395,
        Rotation2d.fromRadians(Math.PI)
    );
    private static final double kStartPoseReadyTranslationToleranceMeters = 0.10;
    private static final double kStartPoseReadyHeadingToleranceDegrees = 3.0;

    private final Swerve swerve = new Swerve();
    private final IntakePivot intakePivot = new IntakePivot();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final AutoRoutines autoRoutines = new AutoRoutines(swerve, intakePivot, intakeRollers, floor, feeder, shooter, hood, hanger, limelight);
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve, intakePivot, intakeRollers, floor, feeder, shooter, hood, hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        // On teleop start: if vision never found 2 AprilTags, reset to known starting pose based on alliance.
        // Blue alliance: x=3, y=4, facing 0° (blue hub). Red alliance (or unknown): x=13, y=4, facing 180° (red hub).
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            if (!limelight.hasInitializedPose()) {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                    swerve.resetPoseAndGyro(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0.0)));
                } else {
                    swerve.resetPoseAndGyro(new Pose2d(13.0, 4.0, Rotation2d.fromDegrees(180.0)));
                }
            }
        }));

        // On enable: home intake pivot (if present) then hanger.
        final Command onEnableCommand = Commands.waitSeconds(0.3).andThen(
            Constants.MechanismPresence.kIntakePivot()
                ? intakePivot.homingCommand().andThen(hanger.homingCommand())
                : hanger.homingCommand());
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(onEnableCommand);

        // Operator: aim-and-shoot, manual shoot (dashboard RPM), and manual shoot presets (Y / X / A = table rows).
        operator.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        operator.rightBumper().whileTrue(subsystemCommands.shootManually());
        operator.y().whileTrue(subsystemCommands.shootWithPreset(
            PrepareShotCommand.FIRST_ROW_SHOT.shooterRPM,
            PrepareShotCommand.FIRST_ROW_SHOT.hoodPosition));
        operator.x().whileTrue(subsystemCommands.shootWithPreset(
            PrepareShotCommand.MID_ROW_SHOT.shooterRPM,
            PrepareShotCommand.MID_ROW_SHOT.hoodPosition));
        operator.a().whileTrue(subsystemCommands.shootWithPreset(
            PrepareShotCommand.THIRD_ROW_SHOT.shooterRPM,
            PrepareShotCommand.THIRD_ROW_SHOT.hoodPosition));
        operator.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        operator.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        // Driver: intake only (pivot + rollers on left trigger, stow on left bumper).
        if (Constants.MechanismPresence.kIntakePivot()) {
            driver.leftTrigger().debounce(0.15).whileTrue(
                Commands.startEnd(
                    () -> {
                        intakePivot.set(IntakePivot.Position.INTAKE);
                        intakeRollers.set(IntakeRollers.Speed.INTAKE);
                    },
                    () -> {
                        intakePivot.set(IntakePivot.Position.STOWED);
                        intakeRollers.set(IntakeRollers.Speed.STOP);
                    },
                    intakePivot, intakeRollers));
            driver.leftBumper().onTrue(intakePivot.runOnce(() -> intakePivot.set(IntakePivot.Position.STOWED)));
        }
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.back().onTrue(Commands.runOnce(() -> manualDriveCommand.toggleFieldCentric()));

        // Start = full reset (origin, 0°). Y = set heading to alliance forward (keep position); use when facing opposing wall.
        driver.start().onTrue(Commands.runOnce(() -> swerve.resetPoseAndGyro(new Pose2d())));
        driver.y().onTrue(Commands.runOnce(() -> swerve.resetHeadingToAllianceForward()));

        // Practice-only: disabled operator action to set a deterministic starting pose.
        // Driver A (while disabled) sets pose to the known Red-hub practice placement.
        RobotModeTriggers.disabled()
            .and(driver.a())
            .onTrue(
                Commands.runOnce(() -> {
                    if (!DriverStation.isFMSAttached()) {
                        swerve.resetPoseAndGyro(Landmarks.practiceRedHubPose());
                        // Keep field-centric reference aligned with the fixed red-hub practice pose.
                        swerve.setOperatorPerspectiveForPracticeRed();
                    }
                }).ignoringDisable(true)
            );
    }

    /**
     * Competition sequence for feeder + shooter only: spin up shooter to dashboard RPM,
     * wait until at speed, then run feeder for {@link Constants.ShooterFeeder#kFeedDurationSeconds}.
     * Use when Hood/Floor/Intake are not in the loop.
     */
    private Command shootSequenceCommand() {
        Command cmd = shooter.spinUpCommand(shooter.getDashboardTargetRPM())
            .andThen(feeder.feedCommand().withTimeout(Constants.ShooterFeeder.kFeedDurationSeconds))
            .finallyDo(() -> shooter.stop());
        cmd.addRequirements(shooter, feeder);
        return cmd;
    }

    private Command updateVisionCommand() {
        return limelight.visionUpdateCommand(swerve);
    }

    public void publishBackupStartPoseDiagnostics() {
        final Pose2d currentPose = swerve.getState().Pose;
        final double expectedX = kBackupAndShootExpectedStartPose.getX();
        final double expectedY = kBackupAndShootExpectedStartPose.getY();
        final double expectedHeadingDeg = kBackupAndShootExpectedStartPose.getRotation().getDegrees();

        final double errorX = currentPose.getX() - expectedX;
        final double errorY = currentPose.getY() - expectedY;
        final double translationError = Math.hypot(errorX, errorY);
        final double headingErrorDeg = currentPose.getRotation()
            .minus(kBackupAndShootExpectedStartPose.getRotation())
            .getDegrees();
        final double absHeadingErrorDeg = Math.abs(headingErrorDeg);

        SmartDashboard.putNumber("Auto/BackupAndShoot/ExpectedStart/X (m)", expectedX);
        SmartDashboard.putNumber("Auto/BackupAndShoot/ExpectedStart/Y (m)", expectedY);
        SmartDashboard.putNumber("Auto/BackupAndShoot/ExpectedStart/Heading (deg)", expectedHeadingDeg);
        SmartDashboard.putNumber("Auto/BackupAndShoot/StartError/X (m)", errorX);
        SmartDashboard.putNumber("Auto/BackupAndShoot/StartError/Y (m)", errorY);
        SmartDashboard.putNumber("Auto/BackupAndShoot/StartError/Translation (m)", translationError);
        SmartDashboard.putNumber("Auto/BackupAndShoot/StartError/Heading (deg)", headingErrorDeg);
        SmartDashboard.putBoolean(
            "Auto/BackupAndShoot/StartPoseReady",
            translationError <= kStartPoseReadyTranslationToleranceMeters
                && absHeadingErrorDeg <= kStartPoseReadyHeadingToleranceDegrees
        );
    }
}
