
package frc.robot.Autonomous.ChargingStationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class LevelOnRampPID extends CommandBase {
  private Swerve mSwerve;
  // private PIDController pitchController = new PIDController(0.03, 0, 0);
  // End with no pitch and stationary
  private double error;
  private double lastPitch;

  private int cnt = 0;
  private boolean backwards = false;

  /** Creates a new LevelOnRamp. */
  public LevelOnRampPID(Swerve Swerve) {
    this.mSwerve = Swerve;
    addRequirements(mSwerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Creates a new LevelOnRamp. */
  public LevelOnRampPID(Swerve Swerve, boolean backwards) {
    this.mSwerve = Swerve;
    addRequirements(mSwerve);
    this.backwards = backwards;
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pitchController.setTolerance(4);
    cnt = 0;
    lastPitch = mSwerve.getPitch().getDegrees();
    mSwerve.enableLockedHeadingMode(true);
    mSwerve.setHeadingGoal(mSwerve.getYaw());
  }

  int counter = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double curPitch = mSwerve.getPitch().getDegrees();
    error = 0. - curPitch;
    double deltaAngle = curPitch - lastPitch;
    
    if (Constants.FMSDETACHED) {
      SmartDashboard.putNumber("balance output", .04 * error);
      SmartDashboard.putNumber("delta angle", deltaAngle);
    }

    mSwerve.getThetaController().setGoal(mSwerve.getHeadingGoal().getRadians());
    double rotationVal = mSwerve.getThetaController().calculate(mSwerve.getYaw().getRadians());
    double thresh = backwards ? 0.275 : .21; 
    // change these numbers for tuning :D
    if ((counter == 0 && Math.abs(deltaAngle) <= thresh)
        || Math.abs(error) >= 14.) {

      mSwerve.drive(new Translation2d(.04 * error, 0.0), rotationVal, false, false, Constants.Swerve.maxSpeed);
    } else {
      counter++;
      // counter allows charging station to settle before continued movement
      if (!backwards) {
        if (counter > 10) {
          counter = 0;
        }
      }
      else {
        if (counter > 30) {
          counter = 0;
        }
      }
      
      SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0));
      SwerveModuleState sms2 = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0));

      mSwerve.setModuleStatesDiamond(new SwerveModuleState[] { sms2, sms, sms, sms2 });
    }

    if (Math.abs(error) <= 6.) {
      cnt++;
    } else {
      cnt = 0;

    }
    lastPitch = curPitch;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cnt >= 20;
  }
}