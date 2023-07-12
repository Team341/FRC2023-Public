// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWristSpeed extends CommandBase {
  Wrist mWrist;
  DoubleSupplier mSpeed;

  /** Creates a new ArmSpeed. */
  public SetWristSpeed(Wrist wrist, DoubleSupplier speed) {
    mWrist = wrist;
    mSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (Math.abs(mSpeed.getAsDouble()) < .03) {
      mWrist.setSpeed(0.0);
    } else {
      mWrist.setSpeed(mSpeed.getAsDouble());
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
