// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class FollowTrajectory extends CommandBase {

  Swerve mDrivebase;
  PathPlannerTrajectory mTrajectory;
  Command mSwerveControllerCommand;
  String mTrajectoryName;
  boolean mOptimized;
  boolean isFirstPath;
  private final Timer m_timer = new Timer();

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Swerve drivebase, PathPlannerTrajectory trajectory, String trajectoryName,
      boolean optimized, boolean isFirstPath) {
    this.mDrivebase = drivebase;
    this.mTrajectory = trajectory;
    this.mTrajectoryName = trajectoryName;
    this.mOptimized = optimized;
    this.isFirstPath = isFirstPath;

    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mTrajectory == null)
      return;
    Constants.Drivebase.AUTO_PID_GAINS.updateFromDashboard();

    mSwerveControllerCommand = new SequentialCommandGroup(

        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            mDrivebase.resetOdometry(mTrajectory.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            mTrajectory,
            mDrivebase::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(
                Constants.Drivebase.AUTO_PID_GAINS.kP,
                Constants.Drivebase.AUTO_PID_GAINS.kI,
                Constants.Drivebase.AUTO_PID_GAINS.kD),
            // Y Controller
            new PIDController(
                Constants.Drivebase.AUTO_PID_GAINS.kP,
                Constants.Drivebase.AUTO_PID_GAINS.kI,
                Constants.Drivebase.AUTO_PID_GAINS.kD), 
            // Turn controller
            new PIDController(Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                Constants.Drivebase.THETA_CONTROLLER_GAINS.kI, Constants.Drivebase.THETA_CONTROLLER_GAINS.kD),
            mDrivebase::setModuleStates,
            false,
            mDrivebase));
    // mSwerveControllerCommand = autoBuilder.fullAuto(mTrajectory);

    mSwerveControllerCommand.initialize();

    // Are these necessary/helpful?
    // Is there some way to move these to only be run if detailedLogging is enabled?
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mTrajectory == null)
      return;

    // Run the autonomous command
    mSwerveControllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (mTrajectory == null)
      return;

    mSwerveControllerCommand.end(interrupted);
    // mDrivebase.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    mDrivebase.drive(new Translation2d(), 0.0, false, false, Constants.Swerve.maxSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (mTrajectory == null) || mSwerveControllerCommand.isFinished();
  }

}
