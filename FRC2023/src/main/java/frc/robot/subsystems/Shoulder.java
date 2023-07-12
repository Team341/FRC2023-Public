// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.DaisyMath;

public class Shoulder extends SubsystemBase {

  public static Shoulder instance = null;

  private static TalonFX mShoulderMasterMotor;
  private static TalonFX mShoulderSlaveMotor;

  private static CANCoder mShoulderEncoder;

  private PIDController mShoulderController;

  /** Creates a new Shoulder. */
  public Shoulder() {
    mShoulderEncoder = new CANCoder(Constants.Shoulder.CANCODER_PORT, Constants.CANIVORE_NAME);

    mShoulderMasterMotor = new TalonFX(Constants.Shoulder.MASTER_MOTOR_PORT, Constants.CANIVORE_NAME);
    mShoulderSlaveMotor = new TalonFX(Constants.Shoulder.SLAVE_MOTOR_PORT, Constants.CANIVORE_NAME);

    /* Factory Default all hardware to prevent unexpected behaviour */
    mShoulderMasterMotor.configFactoryDefault();
    mShoulderSlaveMotor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    mShoulderMasterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Shoulder.kPIDLoopIdx,
        Constants.Shoulder.kTimeoutMs);

    mShoulderSlaveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Shoulder.kPIDLoopIdx,
        Constants.Shoulder.kTimeoutMs);

    mShoulderEncoder.configMagnetOffset(-Constants.Shoulder.CANCODER_ABSOLUTE_POSITION_OFFSET);
    Timer.delay(2.0);
    setShoulderEncoderPosition(-getShoulderAbsolutePosition());
    mShoulderMasterMotor.config_kP(0, Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kP, Constants.Shoulder.kTimeoutMs);
    mShoulderMasterMotor.config_kI(0, Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kI, Constants.Shoulder.kTimeoutMs);
    mShoulderMasterMotor.config_kD(0, Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kD, Constants.Shoulder.kTimeoutMs);
    mShoulderMasterMotor.config_kF(0, Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kF, Constants.Shoulder.kTimeoutMs);
    /* Motion Magic Configurations */
    mShoulderMasterMotor.configMotionAcceleration(Constants.Shoulder.ARM_ACCEL, Constants.Shoulder.kTimeoutMs);
    mShoulderMasterMotor.configMotionCruiseVelocity(Constants.Shoulder.ARM_SPEED, Constants.Shoulder.kTimeoutMs);

    /**
     * Set based on what direction you want forward/positive to be.
     * This does not affect sensor phase.
     */
    mShoulderMasterMotor.setInverted(TalonFXInvertType.Clockwise);
    mShoulderSlaveMotor.setInverted(TalonFXInvertType.FollowMaster);

    /* Ensure sensor is positive when output is positive */
    mShoulderMasterMotor.setSensorPhase(true);
    mShoulderSlaveMotor.setSensorPhase(true);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    mShoulderMasterMotor.configNominalOutputForward(0, Constants.Shoulder.kTimeoutMs);
    mShoulderMasterMotor.configNominalOutputReverse(0, Constants.Shoulder.kTimeoutMs);
    mShoulderMasterMotor.configPeakOutputForward(1, Constants.Shoulder.kTimeoutMs);
    mShoulderMasterMotor.configPeakOutputReverse(-1, Constants.Shoulder.kTimeoutMs);

    mShoulderSlaveMotor.configNominalOutputForward(0, Constants.Shoulder.kTimeoutMs);
    mShoulderSlaveMotor.configNominalOutputReverse(0, Constants.Shoulder.kTimeoutMs);
    mShoulderSlaveMotor.configPeakOutputForward(1, Constants.Shoulder.kTimeoutMs);
    mShoulderSlaveMotor.configPeakOutputReverse(-1, Constants.Shoulder.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    mShoulderMasterMotor.configAllowableClosedloopError(0, Constants.Shoulder.kPIDLoopIdx,
        Constants.Shoulder.kTimeoutMs);
    mShoulderSlaveMotor.configAllowableClosedloopError(0, Constants.Shoulder.kPIDLoopIdx,
        Constants.Shoulder.kTimeoutMs);

    mShoulderController = new PIDController(
        Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kP,
        Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kI,
        Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kD);


    mShoulderSlaveMotor.follow(mShoulderMasterMotor);

    // Make tuning the shoulder gains available
    Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.logToDashboard();
    setShoulderGains();


    // Using the cancoder, try to figure out what the offset from the ideal starting
    // position is in order to set the falcon encoders properly
    // so our arm positions will stay the same regardless of where the arm starts on
  }

  public static Shoulder getInstance() {
    if (instance != null) {
      return instance;
    }
    instance = new Shoulder();
    return instance;
  }

  public void setShoulderGains() {
    mShoulderController.setPID(
        Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kP,
        Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kI,
        Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kD);
  }

  // Gets the absolute encoder position from the shoulder output shaft.
  public double getShoulderAbsolutePosition() {
    return DaisyMath.boundAngleNeg180to180Degrees(
        mShoulderEncoder.getAbsolutePosition());
  }

  public void ResetShoulderPosition() {
    mShoulderController.reset();
    setShoulderEncoderPosition(0.0);
  }

  // Degrees
  public double getShoulderPosition() {
    // return DaisyMath.boundAngleNeg180to180Degrees(
    // Encoder.getAbsolutePosition());
    return DaisyMath.boundAngleNeg180to180Degrees(
        mShoulderMasterMotor.getSelectedSensorPosition() * Constants.Shoulder.talonToAngle);
  }

  // This is the amount of degrees the shoulder motor encoder needs to be offset
  // by in order to know it's true position
  public static void setShoulderEncoderPosition(double position) {
    mShoulderMasterMotor.setSelectedSensorPosition(position * Constants.Shoulder.SHOULDER_TALON_POSITION_FACTOR);
  }

  public void setShoulderPosition(double degrees) {


    degrees = degrees * Constants.Shoulder.angleToTalon;

    mShoulderMasterMotor.set(TalonFXControlMode.MotionMagic, degrees);

  }

  public void setSpeed(double speedPercent) {
    mShoulderMasterMotor.set(ControlMode.PercentOutput, speedPercent);
  }

  public void updatePIDs() {
    Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.updateFromDashboard();
    mShoulderController.setP(Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kP);
    mShoulderController.setI(Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kI);
    mShoulderController.setD(Constants.Shoulder.SHOULDER_MOTOR_PID_GAINS.kD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("Shoulder/Absolute Encoder", getShoulderAbsolutePosition());
      SmartDashboard.putNumber("Shoulder/Motor Encoder", getShoulderPosition());
    }

  }
}
