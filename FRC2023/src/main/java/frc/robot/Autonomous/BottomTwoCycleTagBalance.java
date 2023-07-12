// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Autonomous.ChargingStationCommands.DriveForwardAndLevel;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class BottomTwoCycleTagBalance extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public BottomTwoCycleTagBalance(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake,  Catapult mCatapult, boolean isBlue) {

                addCommands(

                                new BottomTwoCyclePt1(mDrivebase, mShoulder, mWrist, mBelt, mIntake, mCatapult, isBlue, true),
                                new ParallelDeadlineGroup(new FollowTrajectory(mDrivebase,
                                                isBlue ? Autos.MoveSideBlue : Autos.MoveSideRed,
                                                "Part 1",
                                                true, true),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, true)),

                                new DriveForwardAndLevel(mDrivebase));
        }
}
