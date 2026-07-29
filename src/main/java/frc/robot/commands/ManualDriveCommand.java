package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;

/**
 * Teleop manual drive command for the swerve drivetrain.
 *
 * Default: robot-centric — left stick forward = robot forward (no field reference).
 * Press Back to switch to field-centric — forward = toward opposing alliance wall
 * (so pressing forward can drive the robot backward if it is facing your own wall).
 * Press Back again to return to robot-centric.
 */
public class ManualDriveCommand extends Command {
    private enum State {
        IDLING,
        DRIVING
    }

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private State currentState = State.IDLING;
    private ManualDriveInput previousInput = new ManualDriveInput();

    /** Default false: robot-centric. True after Back: field-centric (forward = toward opposing wall). */
    private boolean useFieldCentric = false;

    public ManualDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
        addRequirements(swerve);
    }

    /**
     * Toggles between robot-centric and field-centric. When switching to field-centric,
     * seeds so forward = toward opposing alliance wall (robot may drive "backward" if facing own wall).
     */
    public void toggleFieldCentric() {
        useFieldCentric = !useFieldCentric;
        if (useFieldCentric) {
            initialize();
            swerve.seedFieldCentric();
        }
    }

    public boolean isFieldCentric() {
        return useFieldCentric;
    }

    @Override
    public void initialize() {
        currentState = State.IDLING;
        previousInput = new ManualDriveInput();
        inputSmoother.reset();
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        if (input.hasRotation() || input.hasTranslation()) {
            currentState = State.DRIVING;
        } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
            currentState = State.IDLING;
        }
        previousInput = input;

        switch (currentState) {
            case IDLING:
                swerve.setControl(idleRequest);
                break;
            case DRIVING:
                if (useFieldCentric) {
                    swerve.setControl(
                        fieldCentricRequest
                            .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                            .withVelocityY(Driving.kMaxSpeed.times(input.left))
                            .withRotationalRate(Driving.kMaxRotationalRate.times(input.rotation))
                    );
                } else {
                    swerve.setControl(
                        robotCentricRequest
                            .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                            .withVelocityY(Driving.kMaxSpeed.times(input.left))
                            .withRotationalRate(Driving.kMaxRotationalRate.times(input.rotation))
                    );
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Default drive command: runs until interrupted
        return false;
    }
}
