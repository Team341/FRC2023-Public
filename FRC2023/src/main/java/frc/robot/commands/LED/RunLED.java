// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.tracking.LimelightInterface;


public class RunLED extends CommandBase {
  LED mLED;
  BooleanSupplier mLock, mIntook, mIsAligning;

  boolean hasBeenAligned;
  LimelightInterface mInterface = LimelightInterface.getInstance();
  double lastIntake = -1.0;
  int cnt = 0;

  /**
   * Updates LED based on game
   * 
   * @param led
   * @param tower
   * @param shooter
   * @param limelight
   */
  public RunLED(LED led, BooleanSupplier lock, BooleanSupplier isAligning,
      BooleanSupplier intook) {
    mLED = led;
    mLock = lock;
    mIsAligning = isAligning;
    mIntook = intook;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!mIsAligning.getAsBoolean()) cnt = 0;
    if (!mIntook.getAsBoolean()) {
      lastIntake = -1.0;
      LimelightInterface.getInstance().setLimeLightLED(0);
    }
    if (!mLock.getAsBoolean() && Timer.getFPGATimestamp() - lastIntake > 2.0)
      mLED.getLED().clearAnimation(0);
      mLED.isAnimating = false;


    if (mIsAligning.getAsBoolean() && LimelightInterface.isAligned) {
      mLED.getLED().setLEDs(0, 255, 0, 0, 0, Constants.LED.LED_STRING_LENGTH);
    } else if (mIsAligning.getAsBoolean() && !LimelightInterface.isAligned) {
      if (LimelightInterface.yError < 0.0) {
        if (cnt > 12) {
          mLED.getLED().setLEDs(255, 0, 0, 0, Constants.LED.RIGHT_START_INDEX, Constants.LED.RIGHT_LED_LENGTH);

        }
        else {
          mLED.getLED().setLEDs(0, 0, 0, 0, Constants.LED.RIGHT_START_INDEX, Constants.LED.RIGHT_LED_LENGTH);

        }
        mLED.getLED().setLEDs(0, 0, 0, 0, Constants.LED.LEFT_START_INDEX, Constants.LED.LEFT_LED_LENGTH);

      } else {
        mLED.getLED().setLEDs(0, 0, 0, 0, Constants.LED.RIGHT_START_INDEX, Constants.LED.RIGHT_LED_LENGTH);
        if (cnt > 12) {
          mLED.getLED().setLEDs(255, 0, 0, 0, Constants.LED.LEFT_START_INDEX, Constants.LED.LEFT_LED_LENGTH);

        }
        else {
          mLED.getLED().setLEDs(0, 0, 0, 0, Constants.LED.LEFT_START_INDEX, Constants.LED.LEFT_LED_LENGTH);

        }

      }
      cnt++;
      cnt %= 24;
    } else if (mLock.getAsBoolean()) {
      mLED.getLED().animate(Constants.LED.WIN_BM, 0);
      mLED.isAnimating = true;


    } else if (mIntook.getAsBoolean() && (Timer.getFPGATimestamp() - lastIntake <= 2.0 || lastIntake == -1)) {
      if (lastIntake == -1.0)
        lastIntake = Timer.getFPGATimestamp();
      mLED.getLED().animate(new StrobeAnimation(0,0,255,0,0.1,Constants.LED.LED_STRING_LENGTH), 0);
      if ((int)((((Timer.getFPGATimestamp()-lastIntake)-(int)(Timer.getFPGATimestamp()-lastIntake))*100))%20<10) {
        LimelightInterface.getInstance().setLimeLightLED(3);
      }
      else {
      LimelightInterface.getInstance().setLimeLightLED(0);

      }
    } else if (Intake.coneMode) {
      mLED.getLED().setLEDs(255, 255, 0, 0, 0, Constants.LED.LED_STRING_LENGTH);

     
    } else {
      mLED.getLED().setLEDs(255, 0, 255, 0, 0, Constants.LED.LED_STRING_LENGTH);
    }
    if (Intake.coneMode) {
      mLED.getLED().setLEDs(255, 255, 0, 0, Constants.LED.CANDLE_START_INDEX, 4);

    } else {
      mLED.getLED().setLEDs(255, 0, 255, 0, Constants.LED.CANDLE_START_INDEX, 4);

    }

  }

 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
