// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.StateMachine;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.OuttakePiece;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Fling extends SequentialCommandGroup {
  /** Creates a new Fling. */
  public Fling(Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
  Intake mIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake, Constants.ArmPoses.STOW, true),
      new ParallelDeadlineGroup(
          new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake, Constants.ArmPoses.INTAKE_POSITION_CUBE, true),

        new SequentialCommandGroup(
          new WaitCommand(0.15),
          new OuttakePiece(mIntake).withTimeout(0.25)
        ))
    );
  }
}
