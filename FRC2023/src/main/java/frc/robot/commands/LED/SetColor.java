// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class SetColor extends CommandBase {

  LED mLED;

  /**
   * Sets the color of the LED
   * @param led
   */
  public SetColor(LED led) {
    // Use addRequirements() here to declare subsystem dependencies.

    mLED = led;
    addRequirements(mLED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    mLED.getLED().animate(new TwinkleOffAnimation(0, 0, 255, 0, 0.8, 63, TwinkleOffPercent.Percent42));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
