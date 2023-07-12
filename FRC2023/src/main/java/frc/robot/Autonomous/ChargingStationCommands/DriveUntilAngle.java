// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.ChargingStationCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveUntilAngle extends CommandBase {
  Swerve mSwerve;
  double mSpeed, mLimitDeg;
  boolean mLessThanLimit;
  int cnt;
  Rotation2d desiredAngle = new Rotation2d();
  /** Creates a new DriveBackUntilRaised. */
  public DriveUntilAngle(Swerve Swerve, double speed, double limitDeg, boolean lessThanLimit) {
    mSwerve = Swerve;
    mSpeed = speed;
    mLimitDeg = limitDeg;
    mLessThanLimit = lessThanLimit;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerve);
  }

  /** Creates a new DriveBackUntilRaised. */
  public DriveUntilAngle(Swerve Swerve, double speed, double limitDeg, boolean lessThanLimit, Rotation2d desired) {
    mSwerve = Swerve;
    mSpeed = speed;
    mLimitDeg = limitDeg;
    mLessThanLimit = lessThanLimit;
    desiredAngle = desired;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerve);
  }
  int cnt_thresh = 1;

   /** Creates a new DriveBackUntilRaised. */
   public DriveUntilAngle(Swerve Swerve, double speed, double limitDeg, boolean lessThanLimit, int cnt_amt) {
    mSwerve = Swerve;
    mSpeed = speed;
    mLimitDeg = limitDeg;
    mLessThanLimit = lessThanLimit;
    cnt_thresh = cnt_amt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSwerve.setHeadingGoal(desiredAngle);
    cnt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mLessThanLimit && mSwerve.getPitch().getDegrees() <= mLimitDeg) {
      ++cnt; 

    }
    else if (!mLessThanLimit && mSwerve.getPitch().getDegrees() >= mLimitDeg) {
      ++cnt;
    }
    mSwerve.drive(new Translation2d(mSpeed, 0.0), mSwerve.getThetaController().calculate(mSwerve.getYaw().getRadians(), mSwerve.getHeadingGoal().getRadians()), false, false, Constants.Swerve.maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cnt >= cnt_thresh;
  }
}
