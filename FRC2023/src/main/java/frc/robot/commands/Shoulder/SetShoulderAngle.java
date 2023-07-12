// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class SetShoulderAngle extends CommandBase {
  Shoulder mShoulder;
  DoubleSupplier mArmGoal;

  /** Creates a new ArmSpeed. */
  public SetShoulderAngle(Shoulder shoulder, DoubleSupplier armGoal) {
    mShoulder = shoulder;
    mArmGoal = armGoal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShoulder.updatePIDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mShoulder.setShoulderPosition(mArmGoal.getAsDouble());// remember that this is in degrees thanks
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShoulder.setSpeed(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
