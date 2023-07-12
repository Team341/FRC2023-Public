package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.SlewRateLimiter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier useSlowSpeed;

    private SlewRateLimiter filterDrive = new SlewRateLimiter(32.0);
    private SlewRateLimiter filterStrafe = new SlewRateLimiter(32.0);

    private BooleanSupplier useRadius;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier useRadius,
            BooleanSupplier useSlowSpeed) {
        this.s_Swerve = s_Swerve;
        this.useRadius = useRadius;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.useSlowSpeed = useSlowSpeed;

        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(s_Swerve);

    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double maxSpeed = Constants.Swerve.maxSpeed;
        double maxRot = Constants.Swerve.maxAngularVelocity;
        
        // 0 angle = 5 m/s, 90 angle = 1 m/s ==>

        // if (Shoulder.getInstance().getShoulderPosition() >= 0) {
        //     double p = Math.max((5 - Shoulder.getInstance().getShoulderPosition() * (5. / 90.) + 1.0),1.75) / maxSpeed;
        //     maxSpeed *= p;
        // }

        if (useSlowSpeed.getAsBoolean() || Shoulder.getInstance().getShoulderPosition() > 80.) {
            // The driver is holding a button to reduce the speed
            maxSpeed *= Constants.Swerve.SLOW_SPEED_MODIFIER;
            maxRot *= Constants.Swerve.SLOW_ROT_SPEED_MODIFIER;
        }

        double translationVal = filterDrive.calculate(
                Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3) * maxSpeed);
        double strafeVal = filterStrafe.calculate(
                Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3) * maxSpeed);
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 3)
                * maxRot;
        if (Math.abs(rotationVal) > 0.) {
            // The driver has moved the rotation joystick, stop trying
            // to hold our heading in a specific direction
            s_Swerve.enableLockedHeadingMode(false);
        }

        if (s_Swerve.isInLockedHeadingMode()) {
            // We are trying to hold the swerve in a specific direction, use the PID
            // controller to do it
            s_Swerve.getThetaController().setGoal(s_Swerve.getHeadingGoal().getRadians());
            rotationVal = s_Swerve.getThetaController().calculate(s_Swerve.getYaw().getRadians());
        }

        /* Drive */
        if (!useRadius.getAsBoolean()) {
            // Drive normally
            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal,
                    !robotCentricSup.getAsBoolean(),
                    true,
                    maxSpeed);
        } else {
            // Drive rotation around our intake to be able to swivel about the gamepiece
            s_Swerve.driveOffset(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal,
                    !robotCentricSup.getAsBoolean(),
                    true,
                    Constants.Swerve.FORWARD_PIVOT_POINT,
                    maxSpeed);

        }
    }
}
