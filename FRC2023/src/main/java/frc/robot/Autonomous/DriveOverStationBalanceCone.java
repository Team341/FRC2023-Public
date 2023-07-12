// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Autonomous.ChargingStationCommands.DriveBackwardAndLevel;
import frc.robot.Autonomous.ChargingStationCommands.DriveForwardAndLevel;
import frc.robot.Autonomous.ChargingStationCommands.DriveFrontOverChargingStation;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.Intake.OuttakePiece;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmStateMachine;

public class DriveOverStationBalanceCone extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public DriveOverStationBalanceCone(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist,
                        BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue, boolean noLock) {
                                Command c = noLock ? new DriveBackwardAndLevel(mDrivebase, true, noLock) : new DriveBackwardAndLevel(mDrivebase, true);
                addCommands(
                                

                                // new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                //                 Constants.ArmPoses.BACKWARDS_HIGH_CUBE, true).withTimeout(2.5),
                                // new OuttakePiece(mIntake).withTimeout(0.25),

                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE, true).withTimeout(2.5),
                                new OuttakePiece(mIntake).withTimeout(0.25),
                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                // new WaitCommand(0.5),
                                                                new DriveFrontOverChargingStation(mDrivebase)),

                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false)

                                ),
                                new ParallelDeadlineGroup(
                                        new FollowTrajectory(mDrivebase,
                                                        isBlue ? Autos.ChargingOneCycleBlue : Autos.ChargingOneCycleRed,
                                                        getName(), false, true),
                                                        new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false)),
                                new ParallelDeadlineGroup(c,
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false))

                );
        }
        /** Creates a new RedBottom2Cycle. */
        public DriveOverStationBalanceCone(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist,
                        BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue, boolean noLock, boolean forwards) {
                                Command c = (noLock && forwards) ? new SequentialCommandGroup(
                                        new ResetGyro(mDrivebase),
                                        new DriveForwardAndLevel(mDrivebase, noLock),
                                        new ResetGyro(mDrivebase, 180.0)
                                ) : new DriveBackwardAndLevel(mDrivebase, true);
                addCommands(
                                

                                // new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                //                 Constants.ArmPoses.BACKWARDS_HIGH_CUBE, true).withTimeout(2.5),
                                // new OuttakePiece(mIntake).withTimeout(0.25),
                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Constants.ArmPoses.BACKWARDS_HIGH_CONE, true).withTimeout(2.5),
                new OuttakePiece(mIntake).withTimeout(0.25),
                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                new DriveFrontOverChargingStation(mDrivebase)),

                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false)

                                ),
                                new ParallelDeadlineGroup(
                                        new FollowTrajectory(mDrivebase,
                                                        isBlue ? Autos.ChargingOneCycleBlue : Autos.ChargingOneCycleRed,
                                                        getName(), false, true),
                                                        new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false)),
                               
                                new ParallelDeadlineGroup(c,
                                                new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                                                Constants.ArmPoses.STOW, false))


                );
        }
}

