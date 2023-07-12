// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.ChargingStationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveFrontOverChargingStation extends SequentialCommandGroup {
  /** Creates a new DriveBackOverChargingStation. */
  public DriveFrontOverChargingStation(Swerve mSwerve) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      
      new DriveUntilAngle(mSwerve, 1.2, -8.0, true),
      new DriveUntilAngle(mSwerve, 0.85, 8.0, false),
      new DriveUntilAngle(mSwerve, 1.2, 2.0, true)

    );
  }
}
