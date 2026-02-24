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
// import frc.robot.commands.AutoRoutines; // Commented out - requires mechanism subsystems
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AimToAprilTagCommand;
import frc.robot.commands.ManualDriveCommand;
// import frc.robot.commands.SubsystemCommands; // Commented out - requires mechanism subsystems
// import frc.robot.subsystems.Feeder; // Commented out - mechanism disconnected
// import frc.robot.subsystems.Floor; // Commented out - mechanism disconnected
// import frc.robot.subsystems.Hanger; // Commented out - mechanism disconnected
// import frc.robot.subsystems.Hood; // Commented out - mechanism disconnected
// import frc.robot.subsystems.Intake; // Commented out - mechanism disconnected
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Shooter; // Commented out - mechanism disconnected
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
    // Mechanism subsystems commented out - mechanisms are disconnected on practice robot
    // private final Intake intake = new Intake();
    // private final Floor floor = new Floor();
    // private final Feeder feeder = new Feeder();
    // private final Shooter shooter = new Shooter();
    // private final Hood hood = new Hood();
    // private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Auto routines commented out - requires mechanism subsystems
    // private final AutoRoutines autoRoutines = new AutoRoutines(
    //     swerve,
    //     intake,
    //     floor,
    //     feeder,
    //     shooter,
    //     hood,
    //     hanger,
    //     limelight
    // );
    // Subsystem commands commented out - requires mechanism subsystems
    // private final SubsystemCommands subsystemCommands = new SubsystemCommands(
    //     swerve,
    //     intake,
    //     floor,
    //     feeder,
    //     shooter,
    //     hood,
    //     hanger,
    //     () -> -driver.getLeftY(),
    //     () -> -driver.getLeftX()
    // );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        // autoRoutines.configure(); // Commented out - requires mechanism subsystems
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

        // Mechanism button bindings commented out - mechanisms are disconnected
        // RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
        //     .onTrue(intake.homingCommand())
        //     .onTrue(hanger.homingCommand());

        // driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        // driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        // driver.leftTrigger().whileTrue(intake.intakeCommand());
        // driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        // driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        // driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
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

        // Aim to hub - rotation only, uses field pose and Landmarks (competition)
        final AimAndDriveCommand aimCommand = new AimAndDriveCommand(swerve);
        driver.rightBumper().whileTrue(aimCommand);
        driver.start().onTrue(Commands.runOnce(() -> swerve.resetPose(new Pose2d())));
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
