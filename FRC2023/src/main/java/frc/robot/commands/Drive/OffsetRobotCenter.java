// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class OffsetRobotCenter extends CommandBase {

  Swerve mDrivebase;
  DoubleSupplier mOffset;

  private final DoubleSupplier mTranslationXSupplier;
  private final DoubleSupplier mTranslationYSupplier;
  private final DoubleSupplier mRotationSupplier;

  /**
   * Creates a new OffsetRobotCenter
   * Drives in a circle around given radius as if that was the new robot center
   * @param drivebase 
   * @param translationXSupplier (meters per second)
   * @param translationYSupplier (meters per second)
   * @param rotationSupplier (radians per second)
   * @param offset (meters)
   */
  public OffsetRobotCenter(
      Swerve drivebase,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier offset) {

    mOffset = offset;

    mTranslationXSupplier = translationXSupplier;
    mTranslationYSupplier = translationYSupplier;
    mRotationSupplier = rotationSupplier;

    mDrivebase = drivebase;
    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Center Offset", mOffset.getAsDouble());

    if (Math.abs(mRotationSupplier.getAsDouble()) < .075 ) {
      
        SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(90.0));

        mDrivebase.setModuleStates(new SwerveModuleState[]{sms, sms, sms, sms},Constants.Swerve.maxSpeed);
    }
    else {
      mDrivebase.driveOffset(
          new Translation2d(mTranslationXSupplier.getAsDouble(), 
                            mTranslationYSupplier.getAsDouble()), 
          mRotationSupplier.getAsDouble(),
          true,
          true,
          mOffset.getAsDouble(),Constants.Swerve.maxSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
