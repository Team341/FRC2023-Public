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
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class BottomTwoCyclePt1 extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public BottomTwoCyclePt1(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue, boolean catapult) {

                addCommands(

                                new LaunchCubeHighStart(mDrivebase, mShoulder, mWrist,
                                                mBelt, mIntake, mCatapult),
                                new ParallelDeadlineGroup(
                                                new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                                // new WaitCommand(0.5),
                                                                                new FollowTrajectory(mDrivebase,
                                                                                                isBlue ? Autos.BlueBottom2CycleTag
                                                                                                                .get(0)
                                                                                                                : Autos.RedBottom2CycleTag
                                                                                                                                .get(0),
                                                                                                "Part 1",
                                                                                                true, true)),
                                                                new IntakePiece(mIntake)),

                                                new SequentialCommandGroup(

                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.STOW, false)
                                                                                .withTimeout(1.),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.INTAKE_POSITION_CUBE,
                                                                                false))

                                ),

                                // new ParallelDeadlineGroup(new IntakePiece(mIntake),
                                // new DriveToPositionAbsolute(mDrivebase, () -> new Pose2d(
                                // isBlue ? Constants.Field.BLUE_BOTTOM_PIECE_FIELD_LOCATION
                                // : Constants.Field.RED_BOTTOM_PIECE_FIELD_LOCATION,
                                // new Rotation2d())),
                                // new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                // Constants.ArmPoses.INTAKE_POSITION_CONE, false))
                                // .withTimeout(3.0),
                                new ParallelDeadlineGroup(
                                                new FollowTrajectory(mDrivebase,
                                                                isBlue ? Autos.BlueBottom2CycleTag
                                                                                .get(1)
                                                                                : Autos.RedBottom2CycleTag
                                                                                                .get(1),
                                                                "Part 4",
                                                                true, true),

                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false))
                                                .withTimeout(2.0),

                                new ParallelCommandGroup(new DriveToPositionAbsolute(mDrivebase,
                                                RobotContainer.getScoringLocation(
                                                                mDrivebase,
                                                                isBlue ? Constants.Field.TAG_TO_SIDE
                                                                                + Units.inchesToMeters(-2.0)
                                                                                : -Constants.Field.TAG_TO_SIDE - Units
                                                                                                .inchesToMeters(3.5),
                                                                isBlue ? Units.inchesToMeters(-6.5)
                                                                                : Units.inchesToMeters(-6.5))),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.1),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE,
                                                                                true)
                                                                                .withTimeout(2.5)))
                                                .withTimeout(3.0),

                                new OuttakePiece(mIntake).withTimeout(0.25)

                );
        }
}
