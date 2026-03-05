package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;

/** Commands that use both intake pivot and rollers. */
public final class IntakeCommands {

    private IntakeCommands() {}

    /** Pivot to INTAKE and run rollers; on release stop rollers. */
    public static Command intakeCommand(IntakePivot pivot, IntakeRollers rollers) {
        Command cmd = Commands.startEnd(
            () -> {
                pivot.set(IntakePivot.Position.INTAKE);
                rollers.set(IntakeRollers.Speed.INTAKE);
            },
            () -> rollers.set(IntakeRollers.Speed.STOP)
        );
        cmd.addRequirements(pivot, rollers);
        return cmd;
    }

    /** Agitate: run rollers and cycle pivot between AGITATE and INTAKE. */
    public static Command agitateCommand(IntakePivot pivot, IntakeRollers rollers) {
        Command cmd = Commands.runOnce(() -> rollers.set(IntakeRollers.Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    Commands.runOnce(() -> pivot.set(IntakePivot.Position.AGITATE)),
                    Commands.waitUntil(pivot::isPositionWithinTolerance),
                    Commands.runOnce(() -> pivot.set(IntakePivot.Position.INTAKE)),
                    Commands.waitUntil(pivot::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                pivot.set(IntakePivot.Position.INTAKE);
                rollers.set(IntakeRollers.Speed.STOP);
            });
        cmd.addRequirements(pivot, rollers);
        return cmd;
    }
}
