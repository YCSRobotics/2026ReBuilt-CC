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
        // Use USB drive if plugged in; fall back to internal RoboRIO storage.
        // RoboRIO 1 may mount USB at /u (documented) or /media/sda1 (observed in practice).
        // Check both; use the first writable one found.
        String logPath = "/home/lvuser/logs";
        for (String candidate : new String[]{"/u", "/media/sda1"}) {
            java.io.File mount = new java.io.File(candidate);
            if (mount.exists() && mount.canWrite()) {
                logPath = candidate + "/logs";
                new java.io.File(logPath).mkdirs();
                break;
            }
        }
        SmartDashboard.putString("LogPath", logPath);

        SignalLogger.setPath(logPath);
        SignalLogger.start();
        DataLogManager.start(logPath);
        DriverStation.startDataLog(DataLogManager.getLog());

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
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
        m_robotContainer.publishBackupStartPoseDiagnostics();
        m_robotContainer.publishPdhData();

        SmartDashboard.putBoolean("Brownout", RobotController.isBrownedOut());
        SmartDashboard.putNumber("BatteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("CommLink", DriverStation.isDSAttached());
    }

    @Override
    public void disabledInit() {
        // Reduce Limelight thermal output while disabled by throttling frame processing
        LimelightHelpers.SetThrottle("limelight", 150);
    }

    @Override
    public void autonomousInit() {
        // Full processing speed for autonomous (throttle = 0)
        LimelightHelpers.SetThrottle("limelight", 0);
    }

    @Override
    public void teleopInit() {
        // Full processing speed for teleop (throttle = 0)
        LimelightHelpers.SetThrottle("limelight", 0);
    }
}
