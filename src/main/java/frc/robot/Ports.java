package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    /**
     * RoboRIO built-in CAN — mechanisms (intake, feeder, shooter, etc.).
     */
    public static final CANBus kRoboRioCANBus = new CANBus("rio");

    /**
     * CANivore the swerve drive uses (TalonFX, CANcoder, Pigeon). Must match the device name in
     * Phoenix Tuner / Tuner X (often {@code "Swerve Bus"} in generated projects).
     */
    public static final CANBus kCANivoreCANBus = new CANBus("Swerve Bus");

    /** Bus the shooter TalonFXs are on. Use kCANivoreCANBus if shooter is on a CANivore. */
    public static final CANBus kShooterCANBus = kRoboRioCANBus;

    // CAN IDs - Mechanisms organized from lowest to highest (50s range)
    public static final int kIntakeRollers = 50;      // Intake rollers (lowest)
    public static final int kIntakePivot = 51;
    public static final int kFloor = 52;              // Floor mechanism
    public static final int kFeeder = 53;              // Feeder
    public static final int kShooterLeft = 54;
    public static final int kShooterMiddle = 55;
    public static final int kShooterRight = 56;
    public static final int kHanger = 57;               // Hanger (highest)

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
