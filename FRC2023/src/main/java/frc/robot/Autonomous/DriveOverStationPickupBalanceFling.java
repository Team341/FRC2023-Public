// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.Fling;

public class DriveOverStationPickupBalanceFling extends SequentialCommandGroup {
        /** Creates a new RedBottom2Cycle. */
        public DriveOverStationPickupBalanceFling(Swerve mDrivebase, Shoulder mShoulder, Wrist mWrist,
                        BeltDrivenExtension mBelt,
                        Intake mIntake, Catapult mCatapult, boolean isBlue) {

                addCommands(
                                new DriveOverStationPickupBalance(mDrivebase, mShoulder, mWrist, mBelt, mIntake, mCatapult, isBlue, true, true),
                                
                                new Fling(mShoulder, mWrist, mBelt, mIntake),
                                new WaitCommand(20.0)

                );
        }
}
