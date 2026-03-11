package frc.robot.subsystems;

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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Neo2;
import frc.robot.Ports;

/**
 * Feeder subsystem: REV Neo 2.0 on SPARK Max.
 * Uses REVLib 2026 config API: SparkMax, SparkMaxConfig, getClosedLoopController(), setSetpoint(..., SparkBase.ControlType.kVelocity).
 * Config is built with separate statements (not fluent chain) so the variable stays SparkMaxConfig; then motor.configure(config, ResetMode, PersistMode).
 */
public class Feeder extends SubsystemBase {
    public enum Speed {
        FEED(5000);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private static final int kSmartCurrentLimitAmps = 40;

    private final SparkMax motor;
    private final SparkClosedLoopController closedLoop;
    private final RelativeEncoder encoder;

    public Feeder() {
        if (Constants.MechanismPresence.kFeeder()) {
            motor = new SparkMax(Ports.kFeeder, SparkLowLevel.MotorType.kBrushless);
            final double kV = 12.0 / Neo2.kFreeSpeed.in(RotationsPerSecond);
            final SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(SparkBaseConfig.IdleMode.kCoast);
            config.inverted(false);
            config.smartCurrentLimit(kSmartCurrentLimitAmps);
            config.closedLoop
                .p(6e-5)
                .i(0)
                .d(0)
                .outputRange(-1, 1);
            config.closedLoop.feedForward.kV(kV);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            closedLoop = motor.getClosedLoopController();
            encoder = motor.getEncoder();
        } else {
            motor = null;
            closedLoop = null;
            encoder = null;
        }
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        if (closedLoop != null) {
            closedLoop.setSetpoint(speed.angularVelocity().in(RPM), SparkBase.ControlType.kVelocity);
        }
    }

    public void setPercentOutput(double percentOutput) {
        if (motor != null) motor.set(percentOutput);
    }

    /** Feed using full duty cycle so motor always gets max power (velocity control was not pushing). */
    public Command feedCommand() {
        return startEnd(() -> setPercentOutput(0.8), () -> setPercentOutput(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addBooleanProperty("Present", () -> motor != null, null);
        if (motor != null && encoder != null) {
            builder.addDoubleProperty("RPM", encoder::getVelocity, null);
            builder.addDoubleProperty("Output Current", motor::getOutputCurrent, null);
            builder.addDoubleProperty("Applied Output", motor::getAppliedOutput, null);
        }
    }
}
