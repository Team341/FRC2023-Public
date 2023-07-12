// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake instance = null;
  private CANSparkMax mIntakeMotor;
  private int gamePieceHeldCount = 0;
  private boolean isIntaking = false;
  public static boolean coneMode = true;

  private DigitalInput mBeamBreak = new DigitalInput(Constants.Intake.BEAM_BREAK_PORT);

  /** Creates a new Intake. */
  public Intake() {
    mIntakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    mIntakeMotor.setSmartCurrentLimit(Constants.Intake.INTAKE_MOTOR_CURRENT_LIMIT);
    mIntakeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    mIntakeMotor.setIdleMode(IdleMode.kBrake);

    mIntakeMotor.burnFlash();
  }

  /** Get an instance of Intake. */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public void RunForward() {
    isIntaking = true;
    mIntakeMotor.set(Constants.Intake.RUN_SPEED);
  }

  public void RunReverse() {
    isIntaking = false;
    mIntakeMotor.set(Constants.Intake.REVERSE_SPEED);
  }

  public void Stop() {
    isIntaking = false;
    mIntakeMotor.set(0.0);
  }

  public boolean isIntaking() {
    return isIntaking;
  }

  public boolean getGamePieceBeamBreak() {
    return !mBeamBreak.get();
  }

  public boolean hasGamePiece() {
    if (Intake.coneMode)
      return gamePieceHeldCount >= Constants.Intake.GAME_PIECE_ACQUIRED_COUNT_CONE;
    else
      return gamePieceHeldCount >= Constants.Intake.GAME_PIECE_ACQUIRED_COUNT_CUBE;
    // return false;
  }

  @Override
  public void periodic() {
    if (Constants.FMSDETACHED) {
      SmartDashboard.putBoolean("Intake/Beam Break", getGamePieceBeamBreak());
      SmartDashboard.putBoolean("Wrist/cone mode", coneMode);
      SmartDashboard.putNumber("intake count", gamePieceHeldCount);
      SmartDashboard.putNumber("Intake Motor Output", mIntakeMotor.getOutputCurrent());
    }
    // This method will be called once per scheduler run
    if (getGamePieceBeamBreak()) {
      gamePieceHeldCount++;
    } else {
      gamePieceHeldCount = 0;
    }
  }
}
