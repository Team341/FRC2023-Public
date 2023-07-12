package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimAtPoseCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(8, 0, 0, omegaConstraints);

    private final Swerve Swerve;

    private Supplier<Pose2d> targetPoseSupplier;

    DoubleSupplier forwardAxis;
    DoubleSupplier strafeAxis;

    /**
     * Aims at the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param Swerve
     * @param targetPoseSupplier
     */
    public AimAtPoseCommand(
            Swerve Swerve,
            Supplier<Pose2d> targetPoseSupplier,
            DoubleSupplier forwardAxis,
            DoubleSupplier strafeAxis) {
        this.Swerve = Swerve;
        this.targetPoseSupplier = targetPoseSupplier;

        this.forwardAxis = forwardAxis;
        this.strafeAxis = strafeAxis;

        omegaController.setTolerance(Units.degreesToRadians(0.5));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        var robotPose = Swerve.getPoseAbsolute();
     

        omegaController.reset(robotPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        var robotPose = Swerve.getPoseAbsolute();
        var targetPose = targetPoseSupplier.get();

        var targetAngle =
                targetPose.minus(robotPose).getTranslation().getAngle().rotateBy(Rotation2d.fromDegrees(180));

        omegaController.setGoal(robotPose.getRotation().plus(targetAngle).getRadians());

        var xSpeed = forwardAxis.getAsDouble();
        var ySpeed = strafeAxis.getAsDouble();
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        if (omegaController.atGoal()) omegaSpeed = 0;

        Swerve.drive(new Translation2d(xSpeed, ySpeed), omegaSpeed, false, false, Constants.Swerve.maxSpeed);
    }
}