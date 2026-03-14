package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    /**
     * Direction to hub in operator perspective (0° = operator forward).
     * Converts from getAngle() frame (0° = +X) to operator frame (0° = forward) so the
     * controller does not rotate the robot 90° when the hub is straight ahead.
     */
    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return hubDirectionInBlueAlliancePerspective.rotateBy(Rotation2d.fromDegrees(-90));
        }
        return hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                .withVelocityY(Driving.kMaxSpeed.times(input.left))
                .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
