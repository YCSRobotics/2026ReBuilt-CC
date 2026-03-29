// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import java.util.concurrent.atomic.AtomicBoolean;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class AutoRoutines {
    private final Swerve swerve;
    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        Swerve swerve,
        IntakePivot intakePivot,
        IntakeRollers intakeRollers,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Limelight limelight
    ) {
        this.swerve = swerve;
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(swerve, intakePivot, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
        autoChooser.addRoutine("Outpost and Shoot", this::outpostAndShootRoutine);
        autoChooser.addRoutine("Bump to Collect Fuel", this::bumpToCollectFuelRoutine);
        autoChooser.addRoutine("Backup and Shoot", this::backupAndShootRoutine);
        autoChooser.addRoutine("Back Aim and Shoot", this::backAimAndShootRoutine);
        SmartDashboard.putBoolean("Auto Chooser Published", true);
        SmartDashboard.putString("Auto Chooser Debug", "AutoRoutines.configure() ran");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");
        // Load segments by name from deploy/choreo/OutpostAndDepotTrajectory.traj (no codegen required)
        final AutoTrajectory startToOutpost = routine.trajectory("OutpostAndDepotTrajectory", 0);
        final AutoTrajectory outpostToDepot = routine.trajectory("OutpostAndDepotTrajectory", 1);
        final AutoTrajectory depotToShootingPose = routine.trajectory("OutpostAndDepotTrajectory", 2);
        final AutoTrajectory shootingPoseToTower = routine.trajectory("OutpostAndDepotTrajectory", 3);

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );

        routine.active().onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                intakePivot.runOnce(() -> intakePivot.set(IntakePivot.Position.INTAKE))
            )
        );

        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());

        outpostToDepot.atTimeBeforeEnd(1).onTrue(IntakeCommands.intakeCommand(intakePivot, intakeRollers));
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());

        depotToShootingPose.active().whileTrue(limelight.idle(swerve));
        depotToShootingPose.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(2600),
                hood.positionCommand(0.32)
            )
        );
        depotToShootingPose.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootingPoseToTower.cmd()
            )
        );

        shootingPoseToTower.active().whileTrue(limelight.idle(swerve));

        return routine;
    }

    /**
     * Outpost and Shoot: Waypoint 1 → Waypoint 2 (segment 0), stop, pivot to intake, wait 5 s, then
     * Waypoint 3 (segment 1) and shoot.
     * Uses deploy/choreo/OutpostAndShoot.traj (segment 0: W1→W2, segment 1: W2→W3).
     */
    private AutoRoutine outpostAndShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Shoot");
        final AutoTrajectory startToOutpost = routine.trajectory("OutpostAndShoot", 0);
        final AutoTrajectory outpostToShootPose = routine.trajectory("OutpostAndShoot", 1);
        final double pauseAtWaypoint2Seconds = 5.0;
        final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd(),
                // Waypoint 2: pivot to intake, then hold swerve idle for a full 5 s.
                // deadline(waitSeconds) sets the duration; idle run is cancelled when the wait ends.
                Commands.sequence(
                    intakePivot.runOnce(() -> intakePivot.set(IntakePivot.Position.INTAKE)),
                    Commands.deadline(
                        Commands.waitSeconds(pauseAtWaypoint2Seconds),
                        Commands.run(() -> swerve.setControl(idleRequest), swerve)
                    )
                ),
                outpostToShootPose.cmd()
            )
        );

        // Spin up shooter and hood while approaching shoot pose
        outpostToShootPose.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(PrepareShotCommand.MID_ROW_SHOT.shooterRPM),
                hood.positionCommand(PrepareShotCommand.MID_ROW_SHOT.hoodPosition)
            )
        );
        outpostToShootPose.active().whileTrue(limelight.idle(swerve));

        // When waypoint-3 is complete, shoot (fixed preset; no drivetrain stop gate).
        outpostToShootPose.done().onTrue(
            subsystemCommands.shootWithPreset(
                    PrepareShotCommand.MID_ROW_SHOT.shooterRPM,
                    PrepareShotCommand.MID_ROW_SHOT.hoodPosition
                )
                .withTimeout(10)
        );

        return routine;
    }

    /**
     * Backs up a short distance on {@code deploy/choreo/BackupShoot.traj}, then shoots using the
     * fixed mid-row preset values from the shot table.
     */
    private AutoRoutine backupAndShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Backup and Shoot");
        final AutoTrajectory backup = routine.trajectory("BackupShoot");

        routine.active().onTrue(
            Commands.sequence(
                backup.resetOdometry(),
                backup.cmd()
            )
        );

        backup.done().onTrue(
            subsystemCommands.shootWithPreset(
                    PrepareShotCommand.AUTO_ROW_SHOT.shooterRPM,
                    PrepareShotCommand.AUTO_ROW_SHOT.hoodPosition
                )
                .withTimeout(10)
        );

        return routine;
    }

    /**
     * Backs up on {@code deploy/choreo/BackAimShoot.traj}, then at the trajectory's
     * {@code AimAndShoot} event uses aiming plus distance-based shot preparation.
     */
    private AutoRoutine backAimAndShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Back Aim and Shoot");

        final AutoTrajectory backup = routine.trajectory("BackAimShoot");
        final AtomicBoolean aimAndShootEventSeen = new AtomicBoolean(false);
        final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

        routine.active().onTrue(
            Commands.sequence(
                backup.resetOdometry(),
                backup.cmd()
            )
        );

        backup.atTime("AimAndShoot").onTrue(
            Commands.runOnce(() -> aimAndShootEventSeen.set(true))
        );
        backup.done().onTrue(
            Commands.either(
                Commands.sequence(
                    Commands.run(() -> swerve.setControl(idleRequest), swerve).withTimeout(0.2),
                    subsystemCommands.aimAndShoot().withTimeout(5)
                ),
                Commands.none(),
                aimAndShootEventSeen::get
            )
        );

        return routine;
    }

    /** Bump path: start at Bump → pre-bump → crest → landing bump → forward to balls for collect fuel. */
    private AutoRoutine bumpToCollectFuelRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Bump to Collect Fuel");
        // Load trajectory by name from deploy/choreo/NewPath.traj (no codegen required)
        final AutoTrajectory bumpToBalls = routine.trajectory("NewPath");

        routine.active().onTrue(
            Commands.sequence(
                bumpToBalls.resetOdometry(),
                bumpToBalls.cmd()
            )
        );

        return routine;
    }
}
