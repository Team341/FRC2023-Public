// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Autonomous.ChargingStationCommands.DriveBackwardAndLevel;
import frc.robot.commands.Drive.DriveToPositionAbsolute;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.Intake.OuttakePiece;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class BottomThreeCycleFromTwo extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public BottomThreeCycleFromTwo(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue) {

                addCommands(

                                new BottomTwoCyclePt1(mDrivebase, mShoulder, mWrist, mBelt, mIntake, mCatapult, isBlue,
                                                true),
                                new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                                new WaitCommand(1.0),
                                                                new IntakePiece(mIntake)

                                                ),
                                                new FollowTrajectory(mDrivebase,
                                                                isBlue ? Autos.BlueBottomMidPickup.get(0)
                                                                                : Autos.RedBottomMidPickup.get(0),
                                                                "Part 4",
                                                                true, true),
                                                new SequentialCommandGroup(
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.STOW, false)
                                                                                .withTimeout(1.0),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.INTAKE_POSITION_CONE,
                                                                                false))),
                                new ParallelDeadlineGroup(

                                                new FollowTrajectory(mDrivebase,
                                                                isBlue ? Autos.BlueBottomMidPickup.get(1)
                                                                                : Autos.RedBottomMidPickup.get(1),
                                                                "Part 4",
                                                                true, true),
                                                new SequentialCommandGroup(
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.STOW,
                                                                                false).withTimeout(1.3),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE,
                                                                                false))),
                                new ParallelCommandGroup(
                                                new DriveToPositionAbsolute(mDrivebase,
                                                                RobotContainer.getScoringLocation(mDrivebase,
                                                                                isBlue ? (-Constants.Field.TAG_TO_SIDE
                                                                                                - Units.inchesToMeters(
                                                                                                                2.0))
                                                                                                : (Constants.Field.TAG_TO_SIDE

                                                                                                                + Units.inchesToMeters(
                                                                                                                                -3.0)),
                                                                                isBlue ? Units.inchesToMeters(-6.5)
                                                                                                : Units.inchesToMeters(
                                                                                                                -6.5))),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE,
                                                                true).withTimeout(1.5)

                                ),
                                new OuttakePiece(mIntake).withTimeout(0.35)

                // new ParallelDeadlineGroup(new DriveBackwardAndLevel(mDrivebase,
                // isFinished()),
                // new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                // Constants.ArmPoses.STOW, false))

                // new IntakePiece(mIntake),
                // new DriveToPositionAbsolute(mDrivebase, ()->new Pose2d(new
                // Translation2d(2.0,6.0), new Rotation2d())),
                // new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                // Constants.ArmPoses.BACKWARDS_HIGH_CONE),
                // new OuttakePiece(mIntake),
                // new DriveToPositionAbsolute(mDrivebase, ()->new Pose2d(new Translation2d(6.5,
                // 6.0),new Rotation2d())),

                // new FollowTrajectory(mDrivebase, Autos.BlueBottom3Cycle.get(1), "Part 2",
                // true, true),

                // score
                // new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                // Constants.ArmPoses.BACKWARDS_HIGH),
                // new FollowTrajectory(mDrivebase, Autos.BlueBottom3Cycle.get(0), "Part 1",
                // true, true),
                // new FollowTrajectory(mDrivebase, Autos.BlueBottom3Cycle.get(1), "Part 2",
                // true, false)
                // score

                );
        }
}
