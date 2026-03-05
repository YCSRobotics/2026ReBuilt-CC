package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Neo2;
import frc.robot.Ports;

/**
 * Intake pivot: Neo 2.0 on SPARK Max with REV MAXMotion position control.
 */
public class IntakePivot extends SubsystemBase {
    public enum Position {
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0;
    private static final Angle kPositionTolerance = Degrees.of(5);

    private static final double kP = 5e-5;
    private static final double kI = 1e-6;
    private static final double kD = 0;
    private static final double kIz = 0;
    private static final double kMaxOutput = 1;
    private static final double kMinOutput = -1;
    /** Scale &lt; 1 slows motion for direction check; set to 1.0 after verifying direction. */
    private static final double kSpeedScaleForDirectionCheck = 0.25;
    private static final double kCruiseVelRPM = Neo2.kFreeSpeed.in(RPM) * 0.8 * kSpeedScaleForDirectionCheck;
    private static final double kMaxAccelRPMPerSec = 1500 * kSpeedScaleForDirectionCheck;
    private static final double kAllowedErrDegrees = 5;
    private static final int kSmartCurrentLimitAmps = 40;
    /** Lower value = slower homing so you can stop if pivot moves wrong way. */
    private static final double kHomingPercentOutput = 0.05;

    private final SparkMax motor;
    private final SparkClosedLoopController closedLoop;
    private final RelativeEncoder encoder;
    private double targetPositionDegrees;

    private boolean isHomed = false;

    public IntakePivot() {
        if (Constants.MechanismPresence.kIntakePivot()) {
            motor = new SparkMax(Ports.kIntakePivot, SparkLowLevel.MotorType.kBrushless);
            final SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(SparkBaseConfig.IdleMode.kBrake);
            config.inverted(false);
            config.smartCurrentLimit(kSmartCurrentLimitAmps);
            config.closedLoop
                .p(kP)
                .i(kI)
                .d(kD)
                .iZone(kIz)
                .outputRange(kMinOutput, kMaxOutput);
            config.closedLoop.feedForward.kV(12.0 / Neo2.kFreeSpeed.in(RotationsPerSecond));
            config.closedLoop.maxMotion
                .cruiseVelocity(kCruiseVelRPM)
                .maxAcceleration(kMaxAccelRPMPerSec)
                .allowedProfileError(kAllowedErrDegrees);
            config.encoder
                .positionConversionFactor(360.0 / kPivotReduction)
                .velocityConversionFactor(360.0 / kPivotReduction / 60.0);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            closedLoop = motor.getClosedLoopController();
            encoder = motor.getEncoder();
            targetPositionDegrees = Position.STOWED.angle().in(Degrees);
        } else {
            motor = null;
            closedLoop = null;
            encoder = null;
            targetPositionDegrees = Position.STOWED.angle().in(Degrees);
        }
        SmartDashboard.putData(this);
    }

    /** For use by agitate and other commands that need to wait for position. */
    public boolean isPositionWithinTolerance() {
        if (encoder == null) return true;
        double currentDegrees = encoder.getPosition();
        return Math.abs(currentDegrees - targetPositionDegrees) <= kPositionTolerance.in(Degrees);
    }

    private void setPercentOutput(double percentOutput) {
        if (motor != null) motor.set(percentOutput);
    }

    public void set(Position position) {
        targetPositionDegrees = position.angle().in(Degrees);
        if (closedLoop != null) {
            closedLoop.setSetpoint(targetPositionDegrees, SparkBase.ControlType.kMAXMotionPositionControl);
        }
    }

    public Command homingCommand() {
        if (motor == null || encoder == null) return Commands.none();
        return Commands.sequence(
            Commands.runOnce(() -> setPercentOutput(kHomingPercentOutput)),
            Commands.waitUntil(() -> motor.getOutputCurrent() > 6),
            Commands.runOnce(() -> {
                encoder.setPosition(Position.HOMED.angle().in(Degrees));
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addBooleanProperty("Present", () -> motor != null, null);
        if (motor != null && encoder != null) {
            builder.addDoubleProperty("Angle (degrees)", encoder::getPosition, null);
            builder.addDoubleProperty("Pivot Supply Current", motor::getOutputCurrent, null);
        }
    }
}
