// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachine.ArmPose;
import frc.robot.utilities.AlphaFilter;
import frc.robot.utilities.DaisyMath;
import frc.robot.utilities.PIDGains;

public class Wrist extends SubsystemBase {
  private static Wrist instance = null;
  private CANSparkMax mWristMotor;
  private DutyCycleEncoder mWristEncoder;
  private PIDController mPositionController;
  private ArmFeedforward mArmFeedforward;
  private AlphaFilter mPositionFilter;

  private int unlockedCnt = 0;
  private Solenoid unlockSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Wrist.SOLENOID_PORT);

  /** Creates a new Wrist. */
  public Wrist() {
    mWristMotor = new CANSparkMax(Constants.Wrist.WRIST_MOTOR_PORT, MotorType.kBrushless);
    mWristMotor.restoreFactoryDefaults();

    // mWristMotor.setClosedLoopRampRate(0);
    mWristMotor.setIdleMode(IdleMode.kBrake);
    mWristMotor.setSmartCurrentLimit(Constants.Wrist.CURRENT_LIMIT);

    mWristMotor.getPIDController().setP(Constants.Wrist.WRIST_PID.kP);
    mWristMotor.getPIDController().setOutputRange(Constants.Wrist.WRIST_PID.kMinOutput,
        Constants.Wrist.WRIST_PID.kMaxOutput);
    mWristMotor.getEncoder().setPositionConversionFactor(Constants.Wrist.WRIST_POSITION_CONVERSION_FACTOR);
    mWristMotor.setInverted(true);

    mWristMotor.burnFlash();

    mWristEncoder = new DutyCycleEncoder(Constants.Wrist.WRIST_ENCODER_PORT);

    // mLockServo = new Servo(Constants.Wrist.PWM_SERVO_PORT);
    this.LockWrist();

    mPositionController = new PIDController(Constants.Wrist.WRIST_PID.kP,
        Constants.Wrist.WRIST_PID.kI,
        Constants.Wrist.WRIST_PID.kD);
    // new Constraints(Constants.Wrist.MAX_VELOCITY,
    // Constants.Wrist.MAX_ACCELERATION));
    mPositionController.setTolerance(Constants.Wrist.ANGLE_TOLERANCE);
    mArmFeedforward = new ArmFeedforward(Constants.Wrist.WRIST_PID.kS,
        Constants.Wrist.WRIST_PID.kF,
        Constants.Wrist.WRIST_PID.kV);
    mPositionFilter = new AlphaFilter(Constants.Wrist.WRIST_POSITION_FILTER_ALPHA);
    mWristMotor.getPIDController().setP(Constants.Wrist.WRIST_PID.kP);
    mWristMotor.getPIDController().setI(Constants.Wrist.WRIST_PID.kI);
    mWristMotor.getPIDController().setD(Constants.Wrist.WRIST_PID.kD);
    mWristMotor.getEncoder().setPosition(getWristPosition());
    mWristMotor.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    mWristMotor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.Wrist.ANGLE_TOLERANCE, 0);
    mWristMotor.getPIDController().setSmartMotionMaxAccel(Constants.Wrist.MAX_ACCELERATION, 0);
    mWristMotor.getPIDController().setSmartMotionMaxVelocity(Constants.Wrist.MAX_VELOCITY, 0);
    mWristMotor.getPIDController().setSmartMotionMinOutputVelocity(0, 0);

    // Put the PID gains to smart dashboard so they can be tuned
    Constants.Wrist.WRIST_PID.logToDashboard();
  }

  /** Get an instance of Wrist. */
  public static Wrist getInstance() {
    if (instance == null) {
      instance = new Wrist();
    }
    return instance;
  }

  /* Get Encoder positon */
  public double getWristPosition() {
    return DaisyMath
        .boundAngleNegPiToPiRadians(mWristEncoder.getAbsolutePosition() * Constants.Wrist.THROUGH_BORE_CONV_FACTOR
            - Constants.Wrist.WRIST_ZERO_OFFSET);
  }

  public double getWristAbsolutePosition() {
    return DaisyMath
        .boundAngleNegPiToPiRadians(mWristEncoder.getAbsolutePosition() * Constants.Wrist.THROUGH_BORE_CONV_FACTOR);
  }

  public void updatePIDs() {
    Constants.Wrist.WRIST_PID.updateFromDashboard();
    mPositionController.setP(Constants.Wrist.WRIST_PID.kP);
    mPositionController.setI(Constants.Wrist.WRIST_PID.kI);
    mPositionController.setD(Constants.Wrist.WRIST_PID.kD);
    mWristMotor.getPIDController().setP(Constants.Wrist.WRIST_PID.kP);
    mWristMotor.getPIDController().setP(Constants.Wrist.WRIST_PID.kI);
    mWristMotor.getPIDController().setP(Constants.Wrist.WRIST_PID.kD);

  }

  /* Lock Wrist */
  public void LockWrist() {
    unlockSolenoid.set(false);
  }

  /* Unlock Wrist */
  public void UnlockWrist() {
    unlockSolenoid.set(true);
  }



  /* Get Wrist Lock State */
  public boolean isWristLocked() {
    return !(unlockedCnt > Constants.Wrist.SERVO_UNLOCK_COUNT);
  }

  public void setSpeed(double speed) {
    if (this.isWristLocked()) {
      // Wrist is locked, set speed to zero and unlock the wrist
      this.UnlockWrist();
      speed = 0.0;
    }

    mWristMotor.set(DaisyMath.minmax(speed, -Constants.Wrist.MAX_OUTPUT, Constants.Wrist.MAX_OUTPUT));
  }

  public void Stop() {
    mWristMotor.set(0.0);
    this.LockWrist();
  }

  public void setPIDs(PIDGains PID) {
    mWristMotor.getPIDController().setP(PID.kP);
    mWristMotor.getPIDController().setOutputRange(Constants.Wrist.WRIST_PID.kMinOutput,
        Constants.Wrist.WRIST_PID.kMaxOutput);

    mPositionController.setP(PID.kP);
    mPositionController.setI(PID.kI);
    mPositionController.setD(PID.kD);
    mArmFeedforward = new ArmFeedforward(Constants.Wrist.WRIST_PID.kS, Constants.Wrist.WRIST_PID.kF,
        Constants.Wrist.WRIST_PID.kV);
  }

  private double prevAngle = Double.MAX_VALUE;
  private double beta;

  public void setPosition(ArmPose goalPose, ArmPose currentPose) {
    double position = this.mPositionFilter.calculate(goalPose.wrist.getRadians());
    // We are free to move to the goal position
    mPositionController.setSetpoint(position);
    double speed = mPositionController.calculate(currentPose.wrist.getRadians());

    double error = Math
        .abs(DaisyMath.boundAngleNegPiToPiRadians(goalPose.wrist.getRadians() - currentPose.wrist.getRadians()));
    if (error < Constants.Wrist.ANGLE_TOLERANCE / 2.0) {
      prevAngle = Double.MAX_VALUE;

      this.Stop();
    } else if (error > Constants.Wrist.ANGLE_TOLERANCE) {

      if (this.isWristLocked()) {

        // We want to move, but the wrist is still locked, remove the lock before moving
        // the wrist
        this.UnlockWrist();

        if (prevAngle == Double.MAX_VALUE) {
          prevAngle = currentPose.wrist.getRadians() + Math.toRadians(3.0);
          beta = currentPose.beta.getRadians() + Math.toRadians(3.0);
        }
        mWristMotor.set(mArmFeedforward.calculate(beta, prevAngle) +
            Constants.Wrist.WRIST_PID.kP
                * DaisyMath.boundAngleNegPiToPiRadians(prevAngle -
                    currentPose.wrist.getRadians()));
      } else {

        this.setSpeed(speed);

      }
    }

  }

  public void setPositionSmart(ArmPose goalPose, ArmPose currentPose) {
    if (this.isWristLocked()) {
      this.UnlockWrist();
    } else
      mWristMotor.getPIDController().setReference(
          goalPose.wrist.getRadians() / Constants.Wrist.WRIST_POSITION_CONVERSION_FACTOR, ControlType.kPosition);
  }

  public void setPositionWithoutServo(ArmPose goalPose, ArmPose currentPose) {
    double position = this.mPositionFilter.calculate(goalPose.wrist.getRadians());
    // We are free to move to the goal position
    mPositionController.setSetpoint(position);
    double speed = mPositionController.calculate(currentPose.wrist.getRadians());
    if (mPositionController.atSetpoint()) {
      if (prevAngle == Double.MAX_VALUE) {
        prevAngle = currentPose.wrist.getRadians();
        beta = currentPose.beta.getRadians();
      }

      mWristMotor.set(mArmFeedforward.calculate(beta, prevAngle) + Constants.Wrist.WRIST_PID.kP
          * DaisyMath.boundAngleNegPiToPiRadians(prevAngle - currentPose.wrist.getRadians()));
    } else {
      prevAngle = Double.MAX_VALUE;
      this.setSpeed(speed);
    }
  }

  @Override
  public void periodic() {
    if (unlockSolenoid.get()) {
      unlockedCnt++;
    } else {
      unlockedCnt = 0;
    }

    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("Wrist/Through Bore Encoder", Math.toDegrees(getWristPosition()));
      SmartDashboard.putNumber("real absolute wrist", Math.toDegrees(getWristAbsolutePosition()));
      SmartDashboard.putBoolean("Wrist/Is Locked", this.isWristLocked());
      SmartDashboard.putNumber("Wrist/unlocked count", unlockedCnt);

      SmartDashboard.putNumber("Wrist/Motor Output", mWristMotor.getAppliedOutput());
    }
  }

  public void resetFilter() {
    mPositionFilter.resetWithAngle(getWristPosition());
    mPositionController.reset();
  }

}
