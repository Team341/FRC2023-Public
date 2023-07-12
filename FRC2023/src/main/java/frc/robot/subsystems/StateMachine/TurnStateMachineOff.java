// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.StateMachine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class TurnStateMachineOff extends CommandBase {
  private Shoulder mShoulder;
  private Wrist mWrist;
  private BeltDrivenExtension mBelt;
  /** Creates a new TurnStateMachineOff. */
  public TurnStateMachineOff(Shoulder shoulder, Wrist wrist, BeltDrivenExtension belt) {
    this.mShoulder = shoulder;
    this.mWrist = wrist;
    this.mBelt = belt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShoulder, mWrist, mBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShoulder.setSpeed(0);
    mWrist.Stop();
    mBelt.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShoulder.setSpeed(0);
    mWrist.Stop();
    mBelt.setSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
