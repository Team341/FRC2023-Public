// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.DaisyMath;

// 1640 do this here:https://github.com/FRC1640/2021-Code/blob/main/src/main/java/frc/robot/autonomous/commands/AlignAuton.java

public class TurnToAngle extends CommandBase {

  private Swerve mSwerve;
 
  // PID Values for turn
  // private double kP = Constants.Swerve.ROTATE_PID_GAINS.kP;
  // private double kI = Constants.Swerve.ROTATE_PID_GAINS.kI;
  // private double kD = Constants.Swerve.ROTATE_PID_GAINS.kD;
  // private double kF = Constants.Swerve.VISION_KF;

  private double mTargetAngle;
  // private double maxOutput;

  private double error;

  /**
   * Original align to goal
   * @param Swerve
   * @param leftY left joystick y
   * @param leftX right joystick x 
   */
  public TurnToAngle(Swerve Swerve, Double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = Swerve;
    // maxOutput = 0.35;
    mTargetAngle = targetAngle;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Is the below right? If this isn't driving correctly, I would expect it to be because of this
    // tx is the error
    // Should this be in the execute?
    // targetAngle = DaisyMath.boundAngleNeg180to180Degrees(mSwerve.getAngle().getDegrees() - mLimelightInterface.tx);
    // mSwerve.getThetaController().setGoal(targetAngle);
    // targetAngle = DaisyMath.boundAngleNeg180to180Degrees(mSwerve.getAngle().getDegrees() + mLimelightInterface.tx);
    // mLimelightInterface.mLimelightController.setSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = DaisyMath.boundAngleNeg180to180Degrees(mTargetAngle - mSwerve.getPoseAbsolute().getRotation().getDegrees());
    double rot = Constants.Drivebase.VISION_KF * error / 2.0;//DaisyMath.boundAngleNeg180to180Degrees(mLimelightInterface.mLimelightController.getSetpoint() - mSwerve.getAngle().getDegrees()));

    // SmartDashboard.putNumber("Vision/FF Input", targetAngle - mSwerve.getAngle().getDegrees());
    // SmartDashboard.putNumber("Vision/Target Angle", targetAngle);
    // SmartDashboard.putNumber("Vision/Angle When Calculating", mSwerve.getAngle().getDegrees());

    // I see that instead of multiplying by max angular velocity, 1640 just multiplies by 2, might that be better? Nah.
    // double rot = Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * // Does this need to be multiplied by the max speed?
    //     // mSwerve.getThetaController().calculate(
    //     mLimelightInterface.mLimelightController.calculate(
    //       mSwerve.getAngle().getDegrees()
    //     );
    // rot = 0.0;

    mSwerve.drive( new Translation2d(), rot, false, false,Constants.Swerve.maxSpeed
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.drive(new Translation2d(), 0.0, false, false,Constants.Swerve.maxSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return Math.abs(error) < Constants.Drivebase.VISION_ANGLE_TOLERANCE;
  }
}