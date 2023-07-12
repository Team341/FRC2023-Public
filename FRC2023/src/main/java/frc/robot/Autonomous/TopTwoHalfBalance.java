// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Autonomous.ChargingStationCommands.DriveBackwardAndLevel;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class TopTwoHalfBalance extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public TopTwoHalfBalance(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue) {

                addCommands(
                                new TwoCyclePt1(mDrivebase, mShoulder, mWrist, mBelt, mIntake, mCatapult, isBlue,
                                 Constants.ArmPoses.BACKWARDS_HIGH_CONE),
                                new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                                new WaitCommand(1.),
                                                                new IntakePiece(mIntake)),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                new FollowTrajectory(mDrivebase,
                                                                                isBlue ? Autos.BlueTop2Cycle
                                                                                                .get(3)// TODO: change
                                                                                                       // this :D
                                                                                                : Autos.RedTop2Cycle
                                                                                                                .get(3),
                                                                                "Part 1",
                                                                                true, true)),

                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.INTAKE_POSITION_CUBE,
                                                                false))

                                                .withTimeout(4.0),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new FollowTrajectory(mDrivebase,
                                                                                isBlue ? Autos.BlueTopToCharging
                                                                                                : Autos.RedTopToCharging,
                                                                                "Part 2",
                                                                                true, true),
                                                                new DriveBackwardAndLevel(mDrivebase, true)),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW,
                                                                false)

                                )

                );
        }
}
