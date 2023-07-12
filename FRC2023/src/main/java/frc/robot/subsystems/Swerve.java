package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.tracking.LimelightInterface;
import frc.robot.utilities.DaisyMath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public MK4iSwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private ProfiledPIDController mThetaController;

    private Rotation2d headingSetpoint = new Rotation2d();
    private boolean isLockedHeadingMode = false;

    SwerveDrivePoseEstimator estimator;
    private static Swerve instance;

    /** Get an instance of Intake. */
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANIVORE_NAME);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new MK4iSwerveModule[] {
                new MK4iSwerveModule(0, Constants.Swerve.Mod0.constants),
                new MK4iSwerveModule(1, Constants.Swerve.Mod1.constants),
                new MK4iSwerveModule(2, Constants.Swerve.Mod2.constants),
                new MK4iSwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        mThetaController = new ProfiledPIDController(
                Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
                Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
                Constants.Drivebase.THETA_CONTROLLER_GAINS.kD,
                Constants.Drivebase.kThetaControllerConstraints);
        mThetaController.enableContinuousInput(-Math.PI, Math.PI);
        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        estimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.01, 0.01, 0.01),
                VecBuilder.fill(0.05, 0.05, 0.05));
        resetOdometry(getPose());
    }

    public void setHeadingGoal(Rotation2d heading) {
        headingSetpoint = heading;
    }

    public void enableLockedHeadingMode(boolean locked) {
        isLockedHeadingMode = locked;
    }

    public Rotation2d getHeadingGoal() {
        return headingSetpoint;
    }

    public boolean isInLockedHeadingMode() {
        return isLockedHeadingMode;
    }

    public void invertTurnModules(boolean isInverted) {
        mSwerveMods[0].setTurnMotorInverted(isInverted);
        mSwerveMods[1].setTurnMotorInverted(isInverted);
        mSwerveMods[2].setTurnMotorInverted(isInverted);
        mSwerveMods[3].setTurnMotorInverted(isInverted);
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public void setBrakeMode(boolean breakMode) {
        mSwerveMods[0].setBrakeMode(breakMode);
        mSwerveMods[1].setBrakeMode(breakMode);
        mSwerveMods[2].setBrakeMode(breakMode);
        mSwerveMods[3].setBrakeMode(breakMode);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            double speed) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        if (!DriverStation.isFMSAttached()) {
            double[] currentStates = { mSwerveMods[0].getState().speedMetersPerSecond,
                    mSwerveMods[0].getState().angle.getRadians(),
                    mSwerveMods[1].getState().speedMetersPerSecond, mSwerveMods[1].getState().angle.getRadians(),
                    mSwerveMods[2].getState().speedMetersPerSecond, mSwerveMods[2].getState().angle.getRadians(),
                    mSwerveMods[3].getState().speedMetersPerSecond, mSwerveMods[3].getState().angle.getRadians()
            };
            Logger.getInstance().recordOutput("Drive/SwerveModuleSetpoints", swerveModuleStates);
            Logger.getInstance().recordOutput("Drive/SwerveModuleMeasured", currentStates);
            Logger.getInstance().recordOutput("Drive/Swerve Odometry",
                    Rotation2d.fromDegrees(gyro.getYaw()).getRadians());
            Logger.getInstance().recordOutput("Drive/Test Angle in Radians",
                    swerveOdometry.getPoseMeters().getRotation().getRadians());
        }

    }

    /**
     * Drives in a circle around given radius as if that was the new robot center
     * 
     * @param chassisSpeeds designated states for each swerve module
     * @param radius        radius of offset in meters
     */
    public void driveOffset(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            double radius, double speed) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation),
                new Translation2d(radius, 0.0));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates, double speed) {
        // SmartDashboard.putString("Swerve/er", getPose().toString());

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, speed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStatesDiamond(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (MK4iSwerveModule mod : mSwerveMods) {

            mod.setDesiredStateDiamond(desiredStates[mod.moduleNumber], false);

        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getPoseAbsolute() {
        return estimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {

        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetAbsoluteOdometry(Pose2d pose) {
        estimator.resetPosition(getYaw(), getModulePositions(), pose);

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (MK4iSwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (MK4iSwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void setYaw(double y) {
        gyro.setYaw(y);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * @return theta controller
     */
    public ProfiledPIDController getThetaController() {
        return mThetaController;
    }

    public void resetModulesToAbsolute() {
        for (MK4iSwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }

    }

    public Translation3d getNormalVector3d() {
        return new Translation3d(0, 0, 1).rotateBy(new Rotation3d(
                Units.degreesToRadians(gyro.getRoll()),
                Units.degreesToRadians(gyro.getPitch()),
                0));
    }

    public void addVisionPoseEstimate(Pair<Pose3d, Double> measurement) {
        estimator.addVisionMeasurement(measurement.getFirst().toPose2d(), measurement.getSecond());
    }

    public boolean wheelsAligned(double angle) {
        for (MK4iSwerveModule mod : mSwerveMods) {
            if (Math.abs(DaisyMath.boundAngleNeg180to180Degrees(
                    DaisyMath.boundAngleNeg180to180Degrees(mod.getPosition().angle.getDegrees())
                            - DaisyMath.boundAngleNeg180to180Degrees(angle))) > 5.0) {
                return false;
            }

        }
        return true;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        estimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(),
                getModulePositions());
        var pose = LimelightInterface.getInstance().getRobotPoseInFieldSpace();
        if (pose != null) {
            addVisionPoseEstimate(pose);
        }

        if (Constants.FMSDETACHED) {

            SmartDashboard.putString("Swerve/Pose", getPose().toString());
            SmartDashboard.putString("Swerve/Estimated 2d Pose", getPoseAbsolute().toString());

            SmartDashboard.putNumber("Pitch", getPitch().getDegrees());
            SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        }

        for (MK4iSwerveModule mod : mSwerveMods) {
            if (Constants.FMSDETACHED) {
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
                        mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                        mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                        mod.getState().speedMetersPerSecond);

            }
        }
    }
}