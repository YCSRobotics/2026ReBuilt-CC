// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
// import frc.robot.commands.AutoRoutines;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AimToAprilTagCommand;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
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

    // private final AutoRoutines autoRoutines = new AutoRoutines(swerve, intakePivot, intakeRollers, floor, feeder, shooter, hood, hanger, limelight);
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve, intakePivot, intakeRollers, floor, feeder, shooter, hood, hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        // autoRoutines.configure();
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
        if (Constants.kBringupMode) {
            configureManualDriveBindings();
            configureBringupBindings();
            // Intake pivot homing disabled — causes violent motion; focus on shooter/feeder/floor only.
            // RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            //     .onTrue(Commands.waitSeconds(0.3).andThen(intakePivot.homingCommand()));
            if (Constants.MechanismPresence.kHanger()) {
                RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
                    .onTrue(hanger.homingCommand());
            }
            return;
        }

        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        // On enable: home intake pivot (if present) then hanger.
        final Command onEnableCommand = Commands.waitSeconds(0.3).andThen(
            Constants.MechanismPresence.kIntakePivot()
                ? intakePivot.homingCommand().andThen(hanger.homingCommand())
                : hanger.homingCommand());
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(onEnableCommand);

        driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        // Manual shoot: hold right bumper → spin up shooter (dashboard RPM), then feed; release → stop shooter.
        driver.rightBumper().whileTrue(subsystemCommands.shootManually());

        // Driver left trigger: pivot to INTAKE + run rollers while held; release = STOWED + stop rollers.
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
        driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    /**
     * Bringup: one button per subsystem (operator). Hold one button at a time to verify.
     * Driver stick = swerve. Order: Floor → Intake → Feeder → Shooter → Hanger.
     * Bringup-only constants: {@link Constants.Bringup}. Search codebase for "Bringup" to find all bringup code.
     */
    private void configureBringupBindings() {
        // Bringup: Operator A = Floor feed (while held); release to stop.
        operator.a().whileTrue(floor.runCommand(Floor.Speed.FEED));
        // Bringup: Operator B = Intake rollers only (while held)
        operator.b().whileTrue(intakeRollers.runCommand(IntakeRollers.Speed.INTAKE));
        // Intake pivot disabled — was causing violent back-and-forth; re-enable when pivot is fixed.
        // if (Constants.MechanismPresence.kIntakePivot()) {
        //     final Command pivotBringupCommand = Commands.run(() -> intakePivot.set(IntakePivot.Position.INTAKE), intakePivot)
        //         .finallyDo(() -> intakePivot.set(IntakePivot.Position.STOWED));
        //     operator.leftTrigger().debounce(0.15).whileTrue(pivotBringupCommand);
        // }
        // Operator X = Feeder feed (while held). Only bind when present to avoid CAN traffic for absent device.
        if (Constants.MechanismPresence.kFeeder()) {
            operator.x().whileTrue(feeder.feedCommand());
        }
        // Bringup: Operator Y = Shooter at low RPM (velocity closed-loop). Only bind when present.
        if (Constants.MechanismPresence.kShooter()) {
            operator.y().whileTrue(
                Commands.run(() -> shooter.setRPM(Constants.Bringup.kShooterRPM), shooter).finallyDo(() -> shooter.stop()));
        }
        // Driver right bumper = manual shoot (same as competition): dashboard RPM spin-up then feed; release = stop.
        driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        // Operator LB = Hanger extend; RB = Hanger retract (while held)
        operator.leftBumper().whileTrue(
            Commands.run(() -> hanger.setPercentOutput(0.2), hanger).finallyDo(() -> hanger.setPercentOutput(0)));
        operator.rightBumper().whileTrue(
            Commands.run(() -> hanger.setPercentOutput(-0.2), hanger).finallyDo(() -> hanger.setPercentOutput(0)));
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
        
        // Aim to AprilTag - rotation only, no field/hub (bringup)
        final AimToAprilTagCommand aimToTagCommand = new AimToAprilTagCommand(swerve, limelight);
        operator.rightTrigger().or(driver.rightTrigger()).whileTrue(aimToTagCommand);

        // Aim to hub - rotation only (bringup only). Manual shoot uses right bumper in both modes.
        final AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve);
        if (Constants.kBringupMode) {
            driver.a().whileTrue(aimCommand);  // Driver A = aim to hub when in bringup
        }
        driver.start().onTrue(Commands.runOnce(() -> swerve.resetPose(new Pose2d())));
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
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
}
