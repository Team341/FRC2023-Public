// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmPose;

public class SetWristAngle extends CommandBase {
  /** Creates a new SetWristAngle. */
  Wrist mWrist;
  DoubleSupplier mAngle; // degrees

  public SetWristAngle(Wrist wrist, DoubleSupplier angle) {
    mWrist = wrist;
    mAngle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // mWrist.UnlockWrist();
    mWrist.updatePIDs();
    mWrist.UnlockWrist();
    mWrist.resetFilter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

    
    double shoulderAngle = Shoulder.getInstance().getShoulderAbsolutePosition();
    mWrist.setPosition(new ArmPose(Rotation2d.fromDegrees(shoulderAngle), 0.0,
        Rotation2d.fromDegrees(mAngle.getAsDouble())),
        new ArmPose(Rotation2d.fromDegrees(shoulderAngle), 0.0,
            Rotation2d.fromRadians(mWrist.getWristPosition())));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mWrist.Stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
