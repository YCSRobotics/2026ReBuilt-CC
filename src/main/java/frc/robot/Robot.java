// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Internal flash only. Do not probe /u or /media/sda1: a failed USB port or
        // stale mount makes File.exists()/canWrite() block in the kernel, so Robot()
        // never finishes and the Driver Station shows red code with enable locked out.
        final String logPath = "/home/lvuser/logs";
        new java.io.File(logPath).mkdirs();
        SmartDashboard.putString("LogPath", logPath);

        try {
            SignalLogger.setPath(logPath);
            SignalLogger.start();
            DataLogManager.start(logPath);
            DriverStation.startDataLog(DataLogManager.getLog());
        } catch (RuntimeException ex) {
            // Logging must not prevent the robot program from starting.
            DriverStation.reportError("Failed to start logging: " + ex.getMessage(), ex.getStackTrace());
        }

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        RobotController.setBrownoutVoltage(Volts.of(6.1));
    }
    
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.publishBackupStartPoseDiagnostics();
    }

    @Override
    public void disabledInit() {
        try {
            SignalLogger.stop();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Failed to stop SignalLogger: " + ex.getMessage(), false);
        }
        // 0 = process every frame (needed for hub two-tag calibration while disabled).
        // Restore 150 after calibration to cut Limelight heat on the cart.
        LimelightHelpers.SetThrottle("limelight", 0);
    }

    @Override
    public void autonomousInit() {
        try {
            SignalLogger.start();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Failed to start SignalLogger: " + ex.getMessage(), false);
        }
        // Full processing speed for autonomous (throttle = 0)
        LimelightHelpers.SetThrottle("limelight", 0);
    }

    @Override
    public void teleopInit() {
        try {
            SignalLogger.start();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Failed to start SignalLogger: " + ex.getMessage(), false);
        }
        // Full processing speed for teleop (throttle = 0)
        LimelightHelpers.SetThrottle("limelight", 0);
    }
}
