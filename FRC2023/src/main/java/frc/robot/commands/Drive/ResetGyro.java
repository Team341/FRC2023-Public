// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ResetGyro extends CommandBase {

  Swerve mDrivebase;

  /**
   * Resets gyro
   * @param drivebase 
   */
  public ResetGyro(Swerve drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.

    mDrivebase = drivebase;
    addRequirements(mDrivebase);

  }

  private double mAngle = Double.MAX_VALUE;

  /**
   * Resets gyro
   * @param drivebase 
   */
  public ResetGyro(Swerve drivebase, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    mDrivebase = drivebase;
    mAngle = angle;
    addRequirements(mDrivebase);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // mDrivebase.resetModulesToAbsolute();
    if (mAngle == Double.MAX_VALUE)
    mDrivebase.zeroGyro();
    else {
      mDrivebase.setYaw(mAngle);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
