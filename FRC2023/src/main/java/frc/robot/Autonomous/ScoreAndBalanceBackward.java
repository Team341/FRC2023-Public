// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Autonomous.ChargingStationCommands.DriveBackwardAndLevel;
import frc.robot.commands.Intake.OuttakePiece;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class ScoreAndBalanceBackward extends SequentialCommandGroup {
    /** Creates a new RedBottom2Cycle. */
    public ScoreAndBalanceBackward(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
            Intake mIntake) {

        addCommands(
                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                        Constants.ArmPoses.FORWARD_MID_CUBE, true),
                new OuttakePiece(mIntake).withTimeout(0.25),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new DriveBackwardAndLevel(mDrivebase, true)),
                        new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Constants.ArmPoses.STOW, false))

        );
    }
}
