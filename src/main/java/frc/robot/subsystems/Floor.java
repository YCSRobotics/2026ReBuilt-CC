package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Floor subsystem: REV Neo (or Neo 2) on SPARK Max.
 * Uses percent output for feed/stop; same CAN ID as before (Talon FX slot freed).
 */
public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.55);  // ~3100 RPM equivalent (Neo 2 free 5670 RPM)

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public double getPercentOutput() {
            return percentOutput;
        }
    }

    private static final int kSmartCurrentLimitAmps = 40;

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public Floor() {
        if (Constants.MechanismPresence.kFloor()) {
            motor = new SparkMax(Ports.kFloor, SparkLowLevel.MotorType.kBrushless);
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
        if (motor != null) {
            motor.set(speed.getPercentOutput());
        }
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
    }

    /** Run floor at given speed while held; stop on release. For bringup: use with operator.a() etc. */
    public Command runCommand(Speed speed) {
        return Commands.run(() -> set(speed), this).finallyDo(() -> set(Speed.STOP));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addBooleanProperty("Present", () -> motor != null, null);
        if (motor != null && encoder != null) {
            builder.addDoubleProperty("RPM", encoder::getVelocity, null);
            builder.addDoubleProperty("Output Current", motor::getOutputCurrent, null);
        }
    }
}
