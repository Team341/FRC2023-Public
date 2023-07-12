// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSwerveHeading extends InstantCommand {
  Swerve mDrivebase;
  Rotation2d mSetpoint;

  public SetSwerveHeading(Swerve drivebase, Rotation2d setpoint) {
    mDrivebase = drivebase;
    mSetpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivebase.enableLockedHeadingMode(true);
    mDrivebase.setHeadingGoal(mSetpoint);
  }
}
