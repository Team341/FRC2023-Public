// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.ChargingStationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveFrontOverAndLevel extends SequentialCommandGroup {
  /** Creates a new DriveFrontOverAndLevel. */
  public DriveFrontOverAndLevel(Swerve mSwerve) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new DriveFrontOverChargingStation(mSwerve),
      new DriveBackwardAndLevel(mSwerve, false)
    );
  }
}
