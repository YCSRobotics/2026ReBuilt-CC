// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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

        this.subsystemCommands = new SubsystemCommands(swerve, intakePivot, intakeRollers, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
        autoChooser.addRoutine("Outpost and Shoot", this::outpostAndShootRoutine);
        autoChooser.addRoutine("Bump to Collect Fuel", this::bumpToCollectFuelRoutine);
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

        depotToShootingPose.active().whileTrue(limelight.idle());
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

        shootingPoseToTower.active().whileTrue(limelight.idle());

        return routine;
    }

    /**
     * Outpost and Shoot: start (Waypoint 1) → outpost (stop for human feed) → Waypoint 3 (shoot and stop).
     * Uses deploy/choreo/OutpostAndShootTrajectory.traj (segment 0: start→outpost, segment 1: outpost→shoot pose).
     */
    private AutoRoutine outpostAndShootRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Shoot");
        final AutoTrajectory startToOutpost = routine.trajectory("OutpostAndShootTrajectory", 0);
        final AutoTrajectory outpostToShootPose = routine.trajectory("OutpostAndShootTrajectory", 1);

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );

        // After reaching outpost, wait for human player to feed fuel then drive to shoot pose
        startToOutpost.doneDelayed(0.5).onTrue(
            Commands.sequence(
                Commands.waitSeconds(3.0), // Human player feeds fuel at outpost
                outpostToShootPose.cmd()
            )
        );

        // Spin up shooter and hood while approaching shoot pose
        outpostToShootPose.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(2600),
                hood.positionCommand(0.32)
            )
        );
        outpostToShootPose.active().whileTrue(limelight.idle());

        outpostToShootPose.done().onTrue(
            subsystemCommands.aimAndShoot().withTimeout(5)
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
