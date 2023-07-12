package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.tracking.LimelightInterface;
import java.util.function.Supplier;

public class DriveToPositionAbsolute extends CommandBase {
    private ProfiledPIDController mThetaController = new ProfiledPIDController(
            Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
            Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
            Constants.Drivebase.THETA_CONTROLLER_GAINS.kD,
            Constants.Drivebase.kThetaControllerConstraints);

    private final Swerve swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;

    private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0);


    /**
     * Drives to the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently
     * assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public DriveToPositionAbsolute(Swerve swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;
        mThetaController.setTolerance(Math.toRadians(0.25));
        mThetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    Supplier<Transform2d> mTargetTransformSupplier;

    private Pose2d updatePose() {
        var pose = mTargetTransformSupplier.get();
        if (pose == null)
            return null;

        return new Pose2d(pose.getTranslation(), pose.getRotation());

    }

    /**
     * Drives to the given robot-relative transform on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently
     * assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param swerveDriveSubsystem
     * @param robotToTargetSupplier
     */
    public DriveToPositionAbsolute(
            Swerve swerveDriveSubsystem,
            Supplier<Transform2d> targetTransformSupplier,
            boolean isRobotToTarget) {
        mTargetTransformSupplier = targetTransformSupplier;
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        if (isRobotToTarget)
            this.targetPoseSupplier = this::updatePose;
        else
            this.targetPoseSupplier = () -> swerveDriveSubsystem
                    .getPose()
                    .transformBy(targetTransformSupplier.get().inverse());

        mThetaController.enableContinuousInput(-Math.PI, Math.PI);
        mThetaController.setTolerance(Math.toRadians(0.25));

        addRequirements(swerveDriveSubsystem);
    }

    Pose2d prevTargetPose;

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        LimelightInterface.getInstance().setLimelightPipelineIndex(Constants.Vision.TAG_PIPELINE);
        LimelightInterface.getInstance().setLimeLightLED(1);


        // xController.reset(robotPose.getX(), -robotVelocity.vxMetersPerSecond);
        // yController.reset(robotPose.getY(), -robotVelocity.vyMetersPerSecond);
        xController.setTolerance(Units.inchesToMeters(0.65));
        yController.setTolerance(Units.inchesToMeters(0.5));

        prevTargetPose = null;
        atX = atY = atTheta = false;
        LimelightInterface.isAligned = false;
    }

    private boolean atX, atY, atTheta;

    @Override
    public void execute() {
        Pose2d robotPose = swerveDriveSubsystem.getPoseAbsolute();
        Pose2d targetPose = null;
        Pose2d cur = targetPoseSupplier.get();
        if (cur == null && prevTargetPose == null) {
            return;
        }
        if (prevTargetPose == null) {
            prevTargetPose = cur;

        }
        targetPose = prevTargetPose;
        if (Constants.FMSDETACHED) {

            SmartDashboard.putString("targetPose", targetPose.toString());
            SmartDashboard.putString("robotPose", robotPose.toString());

        }
       
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        mThetaController.setGoal(targetPose.getRotation().getRadians());

        // Drive to the target
        var xSpeed = xController.calculate(robotPose.getX());
        var ySpeed = yController.calculate(robotPose.getY());

        var thetaSpeed = mThetaController.calculate(robotPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        LimelightInterface.yError = yController.getPositionError();
        if (xController.atSetpoint() || atX) {
            xSpeed = 0;
            // atX = true;

        }

        if (yController.atSetpoint() || atY) {
            ySpeed = 0;

        }

        if (mThetaController.atSetpoint() || atTheta) {
            thetaSpeed = 0;

        }

       
        swerveDriveSubsystem.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, false, false,
                Constants.Swerve.maxSpeed);
    }

    @Override
    public boolean isFinished() {

        return xController.atSetpoint() && yController.atSetpoint() && mThetaController.atSetpoint();

    }

    @Override
    public void end(boolean interrupted) {
        LimelightInterface.isAligned = true;

        swerveDriveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false, false, Constants.Swerve.maxSpeed);
    }
}
