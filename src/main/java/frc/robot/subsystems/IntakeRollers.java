package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Intake rollers: Neo 2.0 on SPARK Max. Percent output for in/stop.
 */
public class IntakeRollers extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public double getPercentOutput() {
            return percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    private static final int kSmartCurrentLimitAmps = 40;

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public IntakeRollers() {
        if (Constants.MechanismPresence.kIntakeRollers()) {
            motor = new SparkMax(Ports.kIntakeRollers, SparkLowLevel.MotorType.kBrushless);
            final SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(SparkBaseConfig.IdleMode.kBrake);
            config.inverted(true);
            config.smartCurrentLimit(kSmartCurrentLimitAmps);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            encoder = motor.getEncoder();
        } else {
            motor = null;
            encoder = null;
        }
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        if (motor != null) motor.set(speed.getPercentOutput());
    }

    /** Run rollers while held; stop on release. Requires robot ENABLED (teleop). */
    public Command runCommand(Speed speed) {
        return Commands.run(() -> set(speed), this).finallyDo(() -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addBooleanProperty("Present", () -> motor != null, null);
        if (motor != null && encoder != null) {
            builder.addDoubleProperty("RPM", encoder::getVelocity, null);
            builder.addDoubleProperty("Roller Supply Current", motor::getOutputCurrent, null);
        }
    }
}
