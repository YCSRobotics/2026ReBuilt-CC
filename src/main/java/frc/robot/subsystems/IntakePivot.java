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
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
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
        /** Setpoint in degrees (intake = down). */
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

    private static final double kP = 5e-3; // 0.005 — tuned on REV Hardware Client
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kIz = 0;
    private static final double kMaxOutput = 1;
    private static final double kMinOutput = -1;
    /** Scale &lt; 1 slows motion; increase toward 1.0 only after verifying safe direction and mechanics. */
    private static final double kSpeedScaleForDirectionCheck = 0.08;
    private static final double kCruiseVelRPM = Neo2.kFreeSpeed.in(RPM) * 0.8 * kSpeedScaleForDirectionCheck;
    private static final double kMaxAccelRPMPerSec = 400 * kSpeedScaleForDirectionCheck;
    private static final double kAllowedErrDegrees = 5;
    private static final int kSmartCurrentLimitAmps = 40;
    /** true = motor positive raises pivot (encoder increases toward STOWED 100°). Required so release → set(STOWED) physically returns from intake. */
    private static final boolean kPivotMotorInverted = true;
    /** Lower value = slower homing so you can stop if pivot moves wrong way. */
    private static final double kHomingPercentOutput = 0.05;
    /** Degrees below which we run +output to find home; above this we run -output so we always approach 110° from below. */
    private static final double kHomingMidpointDeg = 55.0;

    private final SparkMax motor;
    private final SparkClosedLoopController closedLoop;
    private final RelativeEncoder encoder;
    private double targetPositionDegrees;

    private final DoubleLogEntry angleDegLog;
    private final DoubleLogEntry velocityRpmLog;
    private final DoubleLogEntry currentAmpLog;
    /** Logged for AdvantageScope: 0=idle, 1=homing, 2=stowed, 3=intake, 4=agitate. */
    private final DoubleLogEntry stateLog;
    private final DoubleLogEntry targetDegLog;
    /** 0=idle, 1=homing, 2=stowed, 3=intake, 4=agitate. Set by set() and homing. */
    private volatile int pivotLogState = 0;

    private boolean isHomed = false;

    public IntakePivot() {
        if (Constants.MechanismPresence.kIntakePivot()) {
            motor = new SparkMax(Ports.kIntakePivot, SparkLowLevel.MotorType.kBrushless);
            final SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(SparkBaseConfig.IdleMode.kBrake);
            config.inverted(kPivotMotorInverted);
            config.smartCurrentLimit(kSmartCurrentLimitAmps);
            config.closedLoop
                .p(kP)
                .i(kI)
                .d(kD)
                .iZone(kIz)
                .outputRange(kMinOutput, kMaxOutput);
            config.closedLoop.feedForward.kV(12.0 / Neo2.kFreeSpeed.in(RotationsPerSecond));
            // MAXMotion: REV standard units — cruiseVelocity in motor RPM, maxAcceleration in motor RPM/s
            config.closedLoop.maxMotion
                .cruiseVelocity(kCruiseVelRPM)
                .maxAcceleration(kMaxAccelRPMPerSec)
                .allowedProfileError(kAllowedErrDegrees);
            // Position in degrees (mechanism); velocity in motor RPM (REV standard)
            config.encoder
                .positionConversionFactor(360.0 / kPivotReduction)
                .velocityConversionFactor(1.0);
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

        angleDegLog = new DoubleLogEntry(DataLogManager.getLog(), "/intake/pivot/angle_deg");
        velocityRpmLog = new DoubleLogEntry(DataLogManager.getLog(), "/intake/pivot/velocity_rpm");
        currentAmpLog = new DoubleLogEntry(DataLogManager.getLog(), "/intake/pivot/current_amp");
        stateLog = new DoubleLogEntry(DataLogManager.getLog(), "/intake/pivot/state");
        targetDegLog = new DoubleLogEntry(DataLogManager.getLog(), "/intake/pivot/target_deg");
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

    /** Stops pivot immediately (0% output). Use on trigger release so motion stops before applying STOWED setpoint. */
    public void stopMoving() {
        setPercentOutput(0);
    }

    public void set(Position position) {
        targetPositionDegrees = position.angle().in(Degrees);
        pivotLogState = position == Position.STOWED || position == Position.HOMED ? 2
            : position == Position.INTAKE ? 3
            : 4; // AGITATE
        if (closedLoop != null) {
            closedLoop.setSetpoint(targetPositionDegrees, SparkBase.ControlType.kMAXMotionPositionControl);
        }
    }

    /**
     * Home by driving toward the 110° (hard stop) side until current > 6 A, then set encoder to 110° and command STOWED.
     * Direction is chosen from current position: below midpoint → +output (increase angle), above midpoint → -output (decrease angle).
     */
    public Command homingCommand() {
        if (motor == null || encoder == null) return Commands.none();
        return Commands.sequence(
            Commands.runOnce(() -> {
                pivotLogState = 1; // homing
                targetPositionDegrees = Position.HOMED.angle().in(Degrees); // for logging: target shows 110° during homing
                double pos = encoder.getPosition();
                setPercentOutput(pos < kHomingMidpointDeg ? kHomingPercentOutput : -kHomingPercentOutput);
            }),
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
    public void periodic() {
        if (motor != null && encoder != null) {
            angleDegLog.append(encoder.getPosition());
            velocityRpmLog.append(encoder.getVelocity());
            currentAmpLog.append(motor.getOutputCurrent());
            stateLog.append(pivotLogState);
            targetDegLog.append(targetPositionDegrees);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addBooleanProperty("Present", () -> motor != null, null);
        if (motor != null && encoder != null) {
            builder.addDoubleProperty("Angle (deg)", encoder::getPosition, null);
            builder.addDoubleProperty("Velocity (RPM)", encoder::getVelocity, null);
            builder.addDoubleProperty("Pivot Supply Current", motor::getOutputCurrent, null);
        }
    }
}
