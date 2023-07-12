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

public class TwoCyclePt1 extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public TwoCyclePt1(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue) {
                this(mDrivebase, mShoulder, mWrist, mBelt, mIntake, mCatapult, isBlue,
                                Constants.ArmPoses.BACKWARDS_MID_CONE);

        }

        public TwoCyclePt1(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue, ArmPose desiredPose) {

                addCommands(

                                // score

                                // new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                // Constants.ArmPoses.BACKWARDS_HIGH_CUBE, true).withTimeout(2.0),
                                // new OuttakePiece(mIntake).withTimeout(0.25),

                                new ParallelDeadlineGroup(
                                                new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.2),
                                                                                new FollowTrajectory(mDrivebase,
                                                                                                isBlue ? Autos.BlueTop2Cycle
                                                                                                                .get(0)
                                                                                                                : Autos.RedTop2Cycle
                                                                                                                                .get(0),
                                                                                                "Part 1",
                                                                                                true, true)),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.75),
                                                                                new IntakePiece(mIntake)

                                                                )),

                                                new LaunchCubeHighStart(mDrivebase, mShoulder, mWrist,
                                                                mBelt, mIntake, mCatapult),

                                                new SequentialCommandGroup(

                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.INTAKE_POSITION_CUBE,
                                                                                false))

                                ).withTimeout(4.0),

                                new ParallelDeadlineGroup(
                                                new FollowTrajectory(mDrivebase, isBlue ? Autos.BlueTop2Cycle
                                                                .get(1)
                                                                : Autos.RedTop2Cycle
                                                                                .get(1),
                                                                "Part 2",
                                                                true, false).withTimeout(isBlue?1.80:1.80),

                                                new SequentialCommandGroup(
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                Constants.ArmPoses.STOW, false)
                                                                                .withTimeout(1.4),
                                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                                desiredPose, false))),

                                // new DriveToPositionAbsolute(mDrivebase,
                                // RobotContainer.getScoringLocation(mDrivebase,
                                // Constants.Field.TAG_TO_SIDE)),

                                new ParallelCommandGroup(
                                                new DriveToPositionAbsolute(mDrivebase,
                                                                RobotContainer.getScoringLocation(mDrivebase,
                                                                                isBlue ? (-Constants.Field.TAG_TO_SIDE
                                                                                                - Units.inchesToMeters(
                                                                                                                3.0))
                                                                                                : (Constants.Field.TAG_TO_SIDE

                                                                                                                - Units.inchesToMeters(
                                                                                                                                2.0)),
                                                                                isBlue ? Units.inchesToMeters(-5.5)
                                                                                                : Units.inchesToMeters(
                                                                                                                -5.5)))
                                                                .withTimeout(2.0),
                                                new SequentialCommandGroup(
                                                                new ArmStateMachine(mShoulder,
                                                                                mWrist, mBelt, mIntake,
                                                                                desiredPose,
                                                                                true)
                                                                                .withTimeout(1.65))),
                                new OuttakePiece(mIntake).withTimeout(0.3)

                // score

                );
        }
}
