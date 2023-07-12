// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class LaunchCubeHighStart extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public LaunchCubeHighStart(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist, BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult) {

                addCommands(

                               
                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                new InstantCommand(() -> mCatapult.launch(), mCatapult),
                                                new WaitCommand(0.2),
                                                new InstantCommand(() -> mCatapult.unLaunch(), mCatapult)))
                                              

                );
        }
}
