
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakePiece extends CommandBase {
    Intake mIntake;
    boolean pulsed = false;
    BooleanSupplier mOverride = null;
    /**
     * Creates a new ArmToAngle.
     * 
     * @param d
     */
    public IntakePiece(Intake intake) {
        mIntake = intake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);

    }

    /**
     * Creates a new ArmToAngle.
     * 
     * @param d
     */
    public IntakePiece(Intake intake, BooleanSupplier override) {
        mIntake = intake;
        mOverride = override;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pulsed = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        mIntake.RunForward();
        pulsed=true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIntake.Stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    if (mOverride == null) return mIntake.hasGamePiece() && pulsed;
    else if (mOverride.getAsBoolean()) return false;
    else return mIntake.hasGamePiece() && pulsed;
    }
}
