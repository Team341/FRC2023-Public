// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.AlphaFilter;
import frc.robot.utilities.DaisyMath;

public class BeltDrivenExtension extends SubsystemBase {

  private static BeltDrivenExtension instance = null;
  private CANSparkMax mBeltMotor;

  private AlphaFilter mPositionFilter;

  private DigitalInput mLimitSwitch;
  public boolean beltZero = false;

  /** Creates a new BeltDrivenExtension. */
  public BeltDrivenExtension() {
    mBeltMotor = new CANSparkMax(Constants.BeltDrive.BELT_MOTOR_PORT, MotorType.kBrushless);
    mBeltMotor.restoreFactoryDefaults();
    mBeltMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    mBeltMotor.setSoftLimit(SoftLimitDirection.kForward, 21);

    mLimitSwitch = new DigitalInput(Constants.BeltDrive.LIMIT_SWITCH_PORT);

    mBeltMotor.setIdleMode(IdleMode.kBrake);
    mBeltMotor.setSmartCurrentLimit(Constants.BeltDrive.CURRENT_LIMIT);

    mBeltMotor.getPIDController().setP(Constants.BeltDrive.BELT_PID.kP);
    mBeltMotor.getPIDController().setI(Constants.BeltDrive.BELT_PID.kI);
    mBeltMotor.getPIDController().setD(Constants.BeltDrive.BELT_PID.kD);
    mBeltMotor.getPIDController().setOutputRange(Constants.BeltDrive.BELT_PID.kMinOutput,
        Constants.Wrist.WRIST_PID.kMaxOutput);
    mBeltMotor.getEncoder().setPositionConversionFactor(Constants.BeltDrive.BELT_POSITION_CONVERSION_FACTOR);

    mBeltMotor.burnFlash();

    mPositionFilter = new AlphaFilter(Constants.BeltDrive.BELT_POSITION_FILTER_ALPHA);
    // Put the PID gains to smart dashboard so they can be tuned
    Constants.BeltDrive.BELT_PID.logToDashboard();
  }

  /** Get an instance of BeltDrive. */
  public static BeltDrivenExtension getInstance() {
    if (instance == null) {
      instance = new BeltDrivenExtension();
    }
    return instance;
  }

  public boolean isLimitSwitchPressed() {
    if (Constants.BeltDrive.INVERT_LIMIT_SWITCH)
      return !mLimitSwitch.get();
    else
      return mLimitSwitch.get();
  }

  /* Returns the belt extension stage length in inches */
  public double getLength() {
    // The analog input returns a value in the range [0, 5]V, scale to be the range
    // of the belt drive [0, 28]in
    return mBeltMotor.getEncoder().getPosition();
  }

  public void zeroExtension() {
    mBeltMotor.getEncoder().setPosition(0.0);
  }


  public void disableReverseSoftLimit() {
    mBeltMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void enableReverseSoftLimit() {
    mBeltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void setSpeed(double speed) {
    mBeltMotor.set(DaisyMath.minmax(speed, -Constants.BeltDrive.MAX_OUTPUT, Constants.BeltDrive.MAX_OUTPUT));
  }

  public void setPosition(double position) {
    double filteredPosition = mPositionFilter.calculate(position);
    if (Math.abs(filteredPosition) <= Constants.BeltDrive.MOVE_SLOW_THRESHOLD) {
      mBeltMotor.getPIDController().setOutputRange(Constants.BeltDrive.HOMING_SPEED, -Constants.BeltDrive.HOMING_SPEED);
    }
    else {
      mBeltMotor.getPIDController().setOutputRange(-Constants.BeltDrive.MAX_OUTPUT, Constants.BeltDrive.MAX_OUTPUT);

    }
    if (Math.abs(filteredPosition) <= Constants.BeltDrive.ZERO_EPSILON && !this.isLimitSwitchPressed()) {
      mBeltMotor.set(Constants.BeltDrive.HOMING_SPEED);
    } else if (!this.isLimitSwitchPressed() || (this.isLimitSwitchPressed() && filteredPosition > getLength())) {
      mBeltMotor.getPIDController().setReference(filteredPosition, CANSparkMax.ControlType.kPosition);

    } else {
      mBeltMotor.set(0.0);
    }
    // Belt motor units should be set to be the same as inches, so the positions
    // should match
  }

  public void updatePIDs() {
    Constants.BeltDrive.BELT_PID.updateFromDashboard();
    mBeltMotor.getPIDController().setP(Constants.BeltDrive.BELT_PID.kP);
    mBeltMotor.getPIDController().setI(Constants.BeltDrive.BELT_PID.kI);
    mBeltMotor.getPIDController().setD(Constants.BeltDrive.BELT_PID.kD);

  }

  // public double getDistance() {

  // return distSens.getRange();
  // }

  @Override
  public void periodic() {
    if (isLimitSwitchPressed()) {
      zeroExtension();
      beltZero = true;
    }
    // This method will be called once per scheduler run
    if (Constants.FMSDETACHED) {

     SmartDashboard.putNumber("Belt Drive/String Pot Length", getLength());
     SmartDashboard.putNumber("Belt Drive/Motor Distance", mBeltMotor.getEncoder().getPosition());
     SmartDashboard.putBoolean("Belt Drive/Limit Switch", isLimitSwitchPressed());

     SmartDashboard.putNumber("Belt Drive/PID Error",
         mBeltMotor.getEncoder().getPosition() - mPositionFilter.getValue());
    }
  }
}
