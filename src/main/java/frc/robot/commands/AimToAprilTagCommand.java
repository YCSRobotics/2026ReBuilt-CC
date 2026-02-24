// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * Rotates the robot to face an AprilTag using the Limelight's tx (horizontal offset).
 * No field pose, hub position, or landmarks required—suitable for bringup.
 */
public class AimToAprilTagCommand extends Command {
    private static final double kTxDeadbandDegrees = 2.0;
    private static final double kP = 8.0; // rad/s per degree of tx (tune for snappier rotation)

    private final Swerve swerve;
    private final Limelight limelight;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    public AimToAprilTagCommand(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        final String name = limelight.getName();
        if (!LimelightHelpers.getTV(name)) {
            swerve.setControl(fieldCentricRequest
                .withVelocityX(Driving.kMaxSpeed.times(0))
                .withVelocityY(Driving.kMaxSpeed.times(0))
                .withRotationalRate(RadiansPerSecond.of(0)));
            return;
        }
        double tx = LimelightHelpers.getTX(name);
        tx = MathUtil.applyDeadband(tx, kTxDeadbandDegrees);
        double omegaRadPerSec = kP * Math.toRadians(tx);
        final double maxRadPerSec = Math.abs(Driving.kMaxRotationalRate.in(RadiansPerSecond));
        omegaRadPerSec = MathUtil.clamp(omegaRadPerSec, -maxRadPerSec, maxRadPerSec);
        final AngularVelocity omega = RadiansPerSecond.of(omegaRadPerSec);

        swerve.setControl(fieldCentricRequest
            .withVelocityX(Driving.kMaxSpeed.times(0))
            .withVelocityY(Driving.kMaxSpeed.times(0))
            .withRotationalRate(omega));
    }
}
