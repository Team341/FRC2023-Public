// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ZeroWheels extends CommandBase {

  private final Swerve mDrivebase;

  /** Creates a new DiamondFormation. */
  public ZeroWheels(Swerve drivebase) {
    mDrivebase = drivebase;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // Prevents robot from budging by setting wheels 45 degrees inwards
    SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    SwerveModuleState sms2 = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    mDrivebase.setModuleStatesDiamond(new SwerveModuleState[]{sms2, sms, sms, sms2});
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDrivebase.wheelsAligned(0.0);
  }
}
