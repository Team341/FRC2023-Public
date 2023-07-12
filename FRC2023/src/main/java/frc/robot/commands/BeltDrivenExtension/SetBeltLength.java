// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BeltDrivenExtension;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BeltDrivenExtension;

public class SetBeltLength extends CommandBase {
  BeltDrivenExtension mBelt;
  DoubleSupplier mArmGoal;

  /** Creates a new ArmSpeed. */
  public SetBeltLength(BeltDrivenExtension belt, DoubleSupplier armGoal) {
    mBelt = belt;
    mArmGoal = armGoal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBelt.updatePIDs();
    // mBelt.resetEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mBelt.setPosition(mArmGoal.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mBelt.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(mBelt.getLength() - mArmGoal.getAsDouble()) <= Constants.BeltDrive.POSITION_TOLERANCE;
  }
}
