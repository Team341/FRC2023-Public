// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.DriveToPositionAbsolute;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.Intake.OuttakePiece;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class BottomThreeCycle extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public BottomThreeCycle(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, boolean isBlue) {

                addCommands(

                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                Constants.ArmPoses.BACKWARDS_HIGH_CUBE, true),
                                new OuttakePiece(mIntake).withTimeout(0.25),
                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                new FollowTrajectory(mDrivebase,
                                                                                isBlue ? Autos.BlueBottom2CycleTag
                                                                                                .get(0)
                                                                                                : Autos.RedBottom2CycleTag
                                                                                                                .get(0),
                                                                                "Part 1",
                                                                                true, true)),
                                                new SequentialCommandGroup(
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.STOW, false)
                                                                                .withTimeout(2.),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.INTAKE_POSITION_CUBE,
                                                                                false))

                                ),

                                new ParallelDeadlineGroup(new IntakePiece(mIntake),
                                                new DriveToPositionAbsolute(mDrivebase, () -> new Pose2d(
                                                                isBlue ? Constants.Field.BLUE_BOTTOM_PIECE_FIELD_LOCATION
                                                                                : Constants.Field.RED_BOTTOM_PIECE_FIELD_LOCATION,
                                                                new Rotation2d())),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.INTAKE_POSITION_CONE, false))
                                                .withTimeout(3.0),
                                new ParallelDeadlineGroup(
                                                new FollowTrajectory(mDrivebase,
                                                                isBlue ? Autos.BlueBottom2Cycle.get(1)
                                                                                : Autos.RedBottom2Cycle.get(1),
                                                                "Part 4",
                                                                true, false),

                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false)),
                                new ParallelCommandGroup(new DriveToPositionAbsolute(mDrivebase,
                                                RobotContainer.getScoringLocation(
                                                                mDrivebase,
                                                                Constants.Field.TAG_TO_SIDE,
                                                                Units.inchesToMeters(
                                                                                1.0))),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE, true))
                                                .withTimeout(3.0),

                                new OuttakePiece(mIntake).withTimeout(0.25),
                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                new FollowTrajectory(mDrivebase,
                                                                                isBlue ? Autos.BlueBottom2Cycle.get(2)
                                                                                                : Autos.RedBottom2Cycle
                                                                                                                .get(2),
                                                                                "Part 4",
                                                                                true, false)

                                                ),
                                                new SequentialCommandGroup(
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.STOW, false)
                                                                                .withTimeout(2.0),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.INTAKE_POSITION_CONE,
                                                                                false))),
                                new ParallelDeadlineGroup(new IntakePiece(mIntake),
                                                new DriveToPositionAbsolute(mDrivebase, () -> new Pose2d(
                                                                isBlue ? Constants.Field.BLUE_MID_BOTTOM_PIECE_FIELD_LOCATION
                                                                                : Constants.Field.RED_MID_BOTTOM_PIECE_FIELD_LOCATION,
                                                                new Rotation2d())),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.INTAKE_POSITION_CONE, false))
                                                .withTimeout(3.0),
                                new ParallelDeadlineGroup(
                                                new FollowTrajectory(mDrivebase,
                                                                Autos.BlueBottom2Cycle.get(3), "Part 4",
                                                                true, false),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false)),
                                new ParallelCommandGroup(new DriveToPositionAbsolute(mDrivebase,
                                                RobotContainer.getScoringLocation(
                                                                mDrivebase,
                                                                -Constants.Field.TAG_TO_SIDE,
                                                                Units.inchesToMeters(
                                                                                1.0))),
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE, true))
                                                .withTimeout(3.0),
                                new OuttakePiece(mIntake).withTimeout(0.25)

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
