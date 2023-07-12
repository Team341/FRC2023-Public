package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREModuleState;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.utilities.DaisyMath;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class MK4iSwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private WPI_TalonFX mAngleMotor;
    private WPI_TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private PIDController turnController = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI,
            Constants.Swerve.angleKD);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    public MK4iSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, Constants.CANIVORE_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new WPI_TalonFX(moduleConstants.angleMotorID, Constants.CANIVORE_NAME);
        // mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID, Constants.CANIVORE_NAME);
        configDriveMotor();
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        lastAngle = getState().angle;
    }

    public void setTurnMotorInverted(boolean isInverted) {
        mAngleMotor.setInverted(isInverted);
    }

    public void setBrakeMode(boolean breakMode) {
        mAngleMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        if (Constants.FMSDETACHED)

            SmartDashboard.putNumber(moduleNumber + " drive voltage", mDriveMotor.getMotorOutputVoltage());

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void setDesiredStateDiamond(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        if (Constants.FMSDETACHED)

            SmartDashboard.putNumber(moduleNumber + " drive voltage", mDriveMotor.getMotorOutputVoltage());
        setAngleDiamond(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        if (Constants.FMSDETACHED)
            SmartDashboard.putNumber(moduleNumber + " Velocity Error",
                    desiredState.speedMetersPerSecond - getState().speedMetersPerSecond);
        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));

        lastAngle = angle;
    }

    private void setAngleDiamond(SwerveModuleState desiredState) {
        Rotation2d angle = desiredState.angle;

        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));

        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public Rotation2d getCanCoderWithOffset() {
        return Rotation2d.fromDegrees(
                DaisyMath.boundAngleNeg180to180Degrees(angleEncoder.getAbsolutePosition() - angleOffset.getDegrees()));
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
                DaisyMath.boundAngleNeg180to180Degrees(getCanCoder().getDegrees() - angleOffset.getDegrees()),
                Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference,
                        Constants.Swerve.driveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference,
                        Constants.Swerve.driveGearRatio),
                getAngle());
    }
}