package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs - Mechanisms organized from floor to top (50s range)
    public static final int kFloor = 50;              // Floor mechanism (lowest)
    public static final int kIntakePivot = 51;        // Intake pivot
    public static final int kIntakeRollers = 52;       // Intake rollers
    public static final int kFeeder = 53;              // Feeder
    public static final int kShooterLeft = 54;         // Shooter left
    public static final int kShooterMiddle = 55;       // Shooter middle
    public static final int kShooterRight = 56;        // Shooter right
    public static final int kHanger = 57;               // Hanger (highest)

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
