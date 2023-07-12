// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BeltDrivenExtension;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BeltDrivenExtension;

public class HomeBeltToZero extends CommandBase {
  BeltDrivenExtension mBelt;
  /** Creates a new HomeBeltToZero. */
  public HomeBeltToZero(BeltDrivenExtension belt) {
    mBelt = belt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBelt.disableReverseSoftLimit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mBelt.setSpeed(Constants.BeltDrive.HOMING_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mBelt.enableReverseSoftLimit();
    mBelt.zeroExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mBelt.isLimitSwitchPressed();
  }
}
