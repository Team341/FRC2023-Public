// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drive.ResetAll;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.LED;
import frc.robot.subsystems.tracking.LimelightInterface;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot  {
  
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private LimelightInterface mLimelightInterface;
  private LED mLED;
  public static boolean logging = false; // true;
  public static boolean detailedLogging = false;//false; // true;
  public static boolean blueRobot = true; 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
 
    ctreConfigs = new CTREConfigs();
    

    try {
      m_robotContainer = new RobotContainer();
    } catch (FileNotFoundException e1) {
      e1.printStackTrace();
    }
    mLimelightInterface = LimelightInterface.getInstance();
    mLimelightInterface.setLimeLightLED(1);
    mLED = LED.getInstance();
    (new ResetAll(Swerve.getInstance())).schedule();


    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser"));

    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to new log
                                         
    }

   // Logger.getInstance().start();
  
    }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
    // mLimelightInterface.run();
    mLimelightInterface.logToDashBoard();
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {Swerve.getInstance().setBrakeMode(false);
    mLED.getLED().clearAnimation(0);
    // Drivebase.getInstance().setAllDriveMode(NeutralMode.Coast);
    mLimelightInterface.setLimeLightLED(1);
    Wrist.getInstance().Stop();
  }

  @Override
  public void disabledPeriodic() {
    mLED.updateAnimation();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    mLED.clear();
    Swerve.getInstance().setBrakeMode(true);
    mLimelightInterface.setLimeLightLED(1);
    mLimelightInterface.setLimelightPipelineIndex(Constants.Vision.TAG_PIPELINE);

    // Drivebase.getInstance().setAllDriveMode(NeutralMode.Brake);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    mLED.clear();
    Swerve.getInstance().setBrakeMode(true);

    mLimelightInterface.setLimeLightLED(1);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
