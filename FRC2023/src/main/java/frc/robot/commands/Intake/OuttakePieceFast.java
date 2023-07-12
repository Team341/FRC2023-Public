
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmPose;

public class OuttakePieceFast extends CommandBase {
    Intake mIntake;
    ArmPose set;

    /**
     * Creates a new ArmToAngle.
     * 
     * @param d
     */
    public OuttakePieceFast(Intake intake, ArmPose setpoint) {
        mIntake = intake;
        set = setpoint;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);

    }

    double start = -1;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(Shoulder.getInstance().getShoulderPosition() - set.shoulder.getDegrees()) <= Constants.Shoulder.SHOULDER_ANGLE_TOLERANCE
        && Math.abs(Wrist.getInstance().getWristPosition()-set.wrist.getRadians())<=Units.degreesToRadians(10.0)
        && Math.abs(BeltDrivenExtension.getInstance().getLength() - set.belt) <= 0.3) {
            if (start == -1)
                start = Timer.getFPGATimestamp();
            mIntake.RunReverse();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIntake.Stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (start == -1)
            return false;
        return Timer.getFPGATimestamp() - start >= 0.25;
    }
}
