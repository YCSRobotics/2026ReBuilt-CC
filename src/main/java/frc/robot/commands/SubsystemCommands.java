package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.IntakeCommands;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        IntakePivot intakePivot,
        IntakeRollers intakeRollers,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        IntakePivot intakePivot,
        IntakeRollers intakeRollers,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this(
            swerve,
            intakePivot,
            intakeRollers,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot() && swerve.isStopped())
                .andThen(feed())
        );
    }

    /**
     * Shoot at a fixed preset (e.g. mid-table) without any aiming/driving.
     * Waits for the drivetrain to be stopped before starting the feeder/floor.
     */
    public Command shootWithPresetAndWaitStopped(double rpm, double hoodPosition) {
        final Command spoolAndSet = Commands.parallel(
            hood.positionCommand(hoodPosition),
            shooter.spinUpCommand(rpm)
        );

        Command cmd = spoolAndSet
            .andThen(Commands.waitUntil(swerve::isStopped))
            .andThen(feed())
            .finallyDo(() -> shooter.stop());

        cmd.addRequirements(shooter, hood, feeder, floor);
        return cmd.handleInterrupt(() -> shooter.stop());
    }

    /** Convenience: fixed mid-table preset (3500 RPM, 0.5 hood) with drivetrain stop gate. */
    public Command shootMidPresetAndWaitStopped() {
        return shootWithPresetAndWaitStopped(
            PrepareShotCommand.MID_ROW_SHOT.shooterRPM,
            PrepareShotCommand.MID_ROW_SHOT.hoodPosition
        );
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> shooter.stop());
    }

    /** Spin up shooter and hood to given preset, wait for both, then feed. Use for manual preset (e.g. first row of table). */
    public Command shootWithPreset(double rpm, double hoodPosition) {
        Command cmd = Commands.parallel(
                hood.positionCommand(hoodPosition),
                shooter.spinUpCommand(rpm))
            .andThen(feed())
            .finallyDo(() -> shooter.stop());
        cmd.addRequirements(shooter, hood, feeder, floor);
        return cmd.handleInterrupt(() -> shooter.stop());
    }

    /** Feeder and floor start together; agitate starts after a short delay; all stop when command ends. */
    private Command feed() {
        return Commands.parallel(
            feeder.feedCommand(),
            floor.feedCommand(),
            Commands.waitSeconds(0.125).andThen(IntakeCommands.agitateCommand(intakePivot, intakeRollers))
        );
    }
}
