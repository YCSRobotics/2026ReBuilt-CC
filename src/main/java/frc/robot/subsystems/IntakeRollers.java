package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Intake rollers: Kraken X60 (Talon FX) on roboRIO CAN. Open-loop voltage (percent) for in/stop.
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

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeRollers() {
        if (Constants.MechanismPresence.kIntakeRollers()) {
            motor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);

            final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                )
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(50))
                        .withSupplyCurrentLimitEnable(true)
                );

            motor.getConfigurator().apply(config);
        } else {
            motor = null;
        }
        SmartDashboard.putData(this);
    }

    public void set(Speed speed) {
        if (motor != null) {
            motor.setControl(voltageRequest.withOutput(speed.voltage()));
        }
    }

    /** Run rollers while held; stop on release. Requires robot ENABLED (teleop). */
    public Command runCommand(Speed speed) {
        return Commands.run(() -> set(speed), this).finallyDo(() -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addBooleanProperty("Present", () -> motor != null, null);
        if (motor != null) {
            builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
            builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
            builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
        }
    }
}
