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
public class LevelOnRamp extends SequentialCommandGroup {
  /** Creates a new LevelOnRamp. */
  public LevelOnRamp(Swerve swerve, boolean backward) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this(swerve, backward, false);
  }
  /** Creates a new LevelOnRamp. */
  public LevelOnRamp(Swerve swerve, boolean backward, boolean noLock) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (noLock) {
      addCommands(
        new SequentialCommandGroup(                                                         
        new LevelOnRampPID(swerve, backward)
        )
  
      );
    }
    else {
      addCommands(
        new SequentialCommandGroup(                                                         
        new LevelOnRampPID(swerve, backward),
        new DiamondFormation(swerve)
        )
  
      );
    }
   
  }
}
