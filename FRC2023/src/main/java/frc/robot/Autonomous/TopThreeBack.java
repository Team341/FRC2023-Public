// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.DriveToPositionAbsolute;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.Intake.OuttakePiece;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmPose;
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class TopThreeBack extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public TopThreeBack(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue, ArmPose scoringPos) {

                addCommands(
                                new TwoCyclePt1(mDrivebase, mShoulder, mWrist, mBelt, mIntake, mCatapult, isBlue, scoringPos),
                                new ParallelDeadlineGroup(
                                                new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.5),
                                                                                new FollowTrajectory(mDrivebase,
                                                                                                isBlue ? Autos.BlueTop2Cycle
                                                                                                                .get(3)
                                                                                                                : Autos.RedTop2Cycle
                                                                                                                                .get(3),
                                                                                                "Part 1",
                                                                                                true, true)),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.5),
                                                                                new IntakePiece(mIntake))),

                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.INTAKE_POSITION_CUBE,
                                                                false))

                                                .withTimeout(4.0),
                                new ParallelDeadlineGroup(new FollowTrajectory(mDrivebase,
                                                isBlue ? Autos.BlueTop2Cycle
                                                                .get(4)
                                                                : Autos.RedTop2Cycle
                                                                                .get(4),
                                                "Part 4",
                                                true, false).withTimeout(isBlue?2.2:2.7),
                                                new SequentialCommandGroup(
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.STOW, false)
                                                                                .withTimeout(1.2),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                scoringPos,
                                                                                false)

                                                )),

                                new ParallelCommandGroup(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                scoringPos, true).withTimeout(isBlue?1.2:1.3),
                                                new SequentialCommandGroup(
                                                                new DriveToPositionAbsolute(mDrivebase,
                                                                                RobotContainer.getScoringLocation(
                                                                                                mDrivebase,
                                                                                                isBlue ? Constants.Field.TAG_TO_SIDE
                                                                                                                + Units.inchesToMeters(
                                                                                                                                2.0)
                                                                                                                : -Constants.Field.TAG_TO_SIDE
                                                                                                                                - Units.inchesToMeters(
                                                                                                                                                5.35),
                                                                                                isBlue?Units.inchesToMeters(
                                                                                                               -3.5):Units.inchesToMeters(
                                                                                                                -2.0)))
                                                                                .withTimeout(1.3)
                                               
                                              )),
                                new OuttakePiece(mIntake).withTimeout(0.3),
                                new ParallelCommandGroup(
                                        new FollowTrajectory(mDrivebase, Autos.ThreeMetersForwardBlue, "", true, true),
                                        new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                Constants.ArmPoses.STOW, false)
                                )

                );

        }
        /** Creates a new RedBottom2Cycle. */
        public TopThreeBack(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue) {

                
                this(mDrivebase, mShoulder, mWrist, mBelt, mIntake, mCatapult, isBlue, Constants.ArmPoses.BACKWARDS_MID_CONE);
        }
}
