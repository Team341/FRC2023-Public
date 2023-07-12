// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.ChargingStationCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBackwardAndLevel extends SequentialCommandGroup {
  /** Creates a new DriveBackAndLevel. */
  public DriveBackwardAndLevel(Swerve mSwerve, boolean teamside) {

    Command c;
    if (teamside) {
      c = new DriveUntilAngle(mSwerve, -1.2, 9, false, 50);

    } else {
      
      
      c = new DriveUntilAngle(mSwerve, -1.2, -9., true, 50);
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        
        c,
        new LevelOnRamp(mSwerve, true)
    );
      
  }
  public DriveBackwardAndLevel(Swerve mSwerve, boolean teamside, boolean noLock) {

    Command c;
    if (teamside) {
      c = new DriveUntilAngle(mSwerve, -1.2, 9, false, 50);

    } else {
      
      
      c = new DriveUntilAngle(mSwerve, -1.2, -9., true, 50);
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        
        c,
        new LevelOnRamp(mSwerve, true, true)
    );
      
  }
  public DriveBackwardAndLevel(Swerve mSwerve, boolean teamside, Rotation2d desired) {

    Command c;
    if (teamside) {
      c = new DriveUntilAngle(mSwerve, -1.2, 9., false, 50); //limit deg = +9

    } else {
      
      
      c = new DriveUntilAngle(mSwerve, -1.2, -9., true, 50);
    }
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        
        c,
        new LevelOnRamp(mSwerve, true)
    );
      
  }
}
