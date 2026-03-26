package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    /** First row of the distance→shot table: 30 in → 2800 RPM, hood 0.3. Use for manual preset shoot (Operator Y). */
    public static final Shot FIRST_ROW_SHOT = new Shot(2800, 0.3);
    /** Mid row: 83 in → 3500 RPM, hood 0.5 (Operator X). */
    public static final Shot MID_ROW_SHOT = new Shot(3400, 0.5);
    /** Third row: 141 in → 3800 RPM, hood 0.7 (Operator A). */
    public static final Shot THIRD_ROW_SHOT = new Shot(3800, 0.7);
    /** First row of the distance→shot table: 30 in → 2800 RPM, hood 0.3. Use for manual preset shoot (Operator Y). */
    public static final Shot Y_KEY_SHOT = new Shot(2800, 0.3);
    /** Mid row: 83 in → 3500 RPM, hood 0.5 (Operator X). */
    public static final Shot X_KEY_SHOT = new Shot(3400, 0.5);
    /** Third row: 141 in → 3800 RPM, hood 0.7 (Operator A). */
    public static final Shot A_KEY_SHOT = new Shot(3800, 0.7);
    /** First row of the distance→shot table: 30 in → 2800 RPM, hood 0.3. Use for manual preset shoot (Operator Y). */
    public static final Shot AUTO_ROW_SHOT = new Shot(3100, 0.4);

    static {
        distanceToShotMap.put(Inches.of(30.0), FIRST_ROW_SHOT);
        distanceToShotMap.put(Inches.of(83.0), MID_ROW_SHOT);
        distanceToShotMap.put(Inches.of(141.0), THIRD_ROW_SHOT);
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter, hood);
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setRPM(shot.shooterRPM);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
