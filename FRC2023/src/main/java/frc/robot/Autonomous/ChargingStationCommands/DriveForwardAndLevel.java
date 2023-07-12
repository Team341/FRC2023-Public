// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.ChargingStationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DiamondFormation;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardAndLevel extends SequentialCommandGroup {
  /** Creates a new DriveBackAndLevel. */
  public DriveForwardAndLevel(Swerve mSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this(mSwerve, false);

  }

  /**
   * Creates a new DriveBackAndLevel.
   * 
   * @param b
   */
  public DriveForwardAndLevel(Swerve mSwerve, boolean noLock) {
    if (noLock) {
      addCommands(
          new DriveUntilAngle(mSwerve, 1.2, -9., true, 50),
          new LevelOnRamp(mSwerve, false, noLock)

      );
    }

    else {
      addCommands(
          new DriveUntilAngle(mSwerve, 1.2, -9., true, 50),
          new LevelOnRamp(mSwerve, false),

          new DiamondFormation(mSwerve));
    }
  }

}
