package frc.robot.subsystems.tracking;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.utilities.DaisyMath;

public class LimelightInterface extends SubsystemBase {
  private static LimelightInterface mInstance;

  public static LimelightInterface getInstance() {
    if (mInstance == null)
      mInstance = new LimelightInterface();
    return mInstance;
  }

  public double tv; // velocity
  public double tx; // x position
  public double ty; // y position
  public double ta; // acceleration
  public double ts; // timestamp

  private double distanceToGoal;
  private int angleOnGoalCount;

  public boolean lockedOnGoal;

  public PIDController mLimelightController;
  public SimpleMotorFeedforward mTurnFeedforward;

  public LimelightInterface() {
    tv = 0.0;
    tx = 0.0;
    ty = 0.0;
    ta = 0.0;
    distanceToGoal = 0.0;
    angleOnGoalCount = 0;
    mLimelightController = new PIDController(
        Constants.Vision.VISION_PID_GAINS.kP,
        Constants.Vision.VISION_PID_GAINS.kI,
        Constants.Vision.VISION_PID_GAINS.kD);

    mTurnFeedforward = new SimpleMotorFeedforward(Constants.Drivebase.TURN_PID_GAINS.kS,
        Constants.Drivebase.VISION_KF);
    if (Robot.logging) {

    }
  }

  /**
   * This is the method for pulling values from LimeLight
   * This is called in the Robot Periodic Method
   */
  public void run() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = -1.0 * NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

    calculateDistance();

    if (Math.abs(tx) <= Constants.Drivebase.VISION_ANGLE_TOLERANCE) {
      angleOnGoalCount++;
    } else {
      angleOnGoalCount = 0;
    }
  }

  /**
   * Calculates the Distance of the robot from the goal
   */
  public void calculateDistance() {
    distanceToGoal = ((Constants.Vision.GOAL_HEIGHT - Constants.Vision.LIMELIGHT_HEIGHT)
        / Math.tan(Math.toRadians(ty + Constants.Vision.INITIAL_ANGLE))) * 12.0;
  }

  /**
   * Sets the LED State of the LimeLight
   * 
   * @param ledState - 1: force off 2: force blink 3: force on
   */
  public void setLimeLightLED(int ledState) {
    NetworkTableInstance.getDefault().getTable(Constants.Vision.LIMELIGHT_NAME).getEntry("ledMode").setNumber(ledState);
  }

  /**
   * Gets the already calculated distance from the goal without updating it
   * 
   * @return distance
   */
  public double getDistance() {
    return distanceToGoal;
  }

  public boolean tooClose() {
    return distanceToGoal <= 0.0;
  }

  public boolean tooFar() {
    return distanceToGoal >= 1.0;
  }

  public void periodic() {
    run();
  }

  /**
   * Method that logs values to dashboard
   */
  public void logToDashBoard() {
    if (Constants.FMSDETACHED)

      SmartDashboard.putNumber("Vision/tx", tx);
    var pose = getRobotPoseInTargetSpace();
    if (pose != null) {
    }

  }

  public double getID() {
    return LimelightHelpers.getFiducialID(Constants.Vision.LIMELIGHT_NAME);

  }

  public Pose3d getRobotPoseInTargetSpace() {
    if (!hasTarget() || getLimelightPipelineIndex() == Constants.Vision.TAPE_PIPELINE)
      return null;
    Pose3d BotPose3d = LimelightHelpers.getBotPose3d_TargetSpace(Constants.Vision.LIMELIGHT_NAME);
    return new Pose3d(BotPose3d.getZ(), -BotPose3d.getX(), BotPose3d.getY(), new Rotation3d(
        BotPose3d.getRotation().getZ(), -BotPose3d.getRotation().getX(), BotPose3d.getRotation().getY()));

  }

  public static boolean isAligned = false;
  public static double yError = 0.0;

  private static boolean isValidPose(Pose2d pose) {
    return DaisyMath.isInRange(pose.getY(), -5, FieldConstants.fieldWidth + 5)
        && DaisyMath.isInRange(pose.getX(), -5, FieldConstants.fieldLength + 5);
  }

  public Pair<Pose3d, Double> getRobotPoseInFieldSpace() {

    if (!hasTarget() || getLimelightPipelineIndex() == Constants.Vision.TAPE_PIPELINE)
      return null;
    double timestamp = Timer.getFPGATimestamp() - ts / 1000.0;
    if (Constants.FMSDETACHED)

      SmartDashboard.putNumber("Vision Timestamp", timestamp);

    if (DriverStation.getAlliance() == Alliance.Red) {

      var pose = new Pair<Pose3d, Double>(LimelightHelpers.getBotPose3d_wpiRed(Constants.Vision.LIMELIGHT_NAME),
          timestamp);
      if (isValidPose(pose.getFirst().toPose2d())) {
        return pose;
      }
    } else {
      var pose = new Pair<Pose3d, Double>(LimelightHelpers.getBotPose3d_wpiBlue(Constants.Vision.LIMELIGHT_NAME),
          timestamp);
      if (isValidPose(pose.getFirst().toPose2d())) {
        return pose;
      }
    }

    return null;

  }

  public void setLimelightPipelineIndex(int idx) {
    LimelightHelpers.setPipelineIndex(Constants.Vision.LIMELIGHT_NAME, idx);
  }

  public int getLimelightPipelineIndex() {
    return (int) LimelightHelpers.getCurrentPipelineIndex(Constants.Vision.LIMELIGHT_NAME);
  }

  

  /**
   * returns if there is a target detected by the limelight
   */
  public boolean hasTarget() {
    return tv > 0;
  }

  /**
   * returns if the robot is aligned with the target
   */
  public boolean alignedToGoal() {
    return hasTarget() && Math.abs(tx) <= Constants.Drivebase.VISION_ANGLE_TOLERANCE && angleOnGoalCount >= 7;
  }

  public boolean notAligned() {
    return Math.abs(tx) > 5.5;
  }

}
