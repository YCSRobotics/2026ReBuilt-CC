package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

/**
 * Shooter subsystem (3 Kraken X60). Velocity closed-loop for spool; coast on stop.
 * Bringup: Operator Y (while held) runs at {@link Constants.Bringup#kShooterRPM}; release stops.
 */
public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 0.0;

    public Shooter() {
        if (Constants.MechanismPresence.kShooter()) {
            leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kShooterCANBus);
            middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kShooterCANBus);
            rightMotor = new TalonFX(Ports.kShooterRight, Ports.kShooterCANBus);
            motors = List.of(leftMotor, middleMotor, rightMotor);
            configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
            configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive);
            configureMotor(rightMotor, InvertedValue.Clockwise_Positive);
        } else {
            leftMotor = null;
            middleMotor = null;
            rightMotor = null;
            motors = List.of();
        }
        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond))
            );
        
        motor.getConfigurator().apply(config);

        // Reduce CAN traffic so Feeder (SPARK Max) on same Rio bus doesn't timeout when shooter runs
        motor.getVelocity().setUpdateFrequency(50);
        motor.getStatorCurrent().setUpdateFrequency(50);
        motor.getSupplyCurrent().setUpdateFrequency(50);
    }

    public void setRPM(double rpm) {
        for (final TalonFX m : motors) {
            m.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        }
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX m : motors) {
            m.setControl(voltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public double getDashboardTargetRPM() {
        return dashboardTargetRPM;
    }

    public boolean isVelocityWithinTolerance() {
        if (motors.isEmpty()) return true;
        return motors.stream().allMatch(m -> {
            final boolean isInVelocityMode = m.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = m.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addBooleanProperty("Present", () -> !motors.isEmpty(), null);
        if (leftMotor != null) initSendable(builder, leftMotor, "Left");
        if (middleMotor != null) initSendable(builder, middleMotor, "Middle");
        if (rightMotor != null) initSendable(builder, rightMotor, "Right");
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
    }
}
