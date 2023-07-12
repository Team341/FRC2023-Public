// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BeltDrivenExtension;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltDrivenExtension;

public class SetBeltSpeed extends CommandBase {
  BeltDrivenExtension mBelt;

  DoubleSupplier mArmGoal;

  /** Creates a new ArmSpeed. */
  public SetBeltSpeed(BeltDrivenExtension belt, DoubleSupplier armGoal) {
    mBelt = belt;
    mArmGoal = armGoal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (Math.abs(mArmGoal.getAsDouble()) < .03) {
      mBelt.setSpeed(0.0);
    } else {
      mBelt.setSpeed(mArmGoal.getAsDouble());
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
