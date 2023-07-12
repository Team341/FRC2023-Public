// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.StateMachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Pose;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.DaisyMath;

public class ArmStateMachine extends CommandBase {
    /** Creates a new moveStateMachine. */
    private Shoulder mShoulder;
    private Wrist mWrist;
    private BeltDrivenExtension mBelt;

    private ArmPose goalPose, original;
    private double deltaShoulder;
    private double deltaWrist;
    private double deltaLength;
    private ArmState shoulderState, wristState, beltState;
    private ArmPose curPose;
    private Pose goal = null;
    private boolean atPose;
    private static ArmPose lastPose = null;

    public ArmStateMachine(Shoulder shoulder, Wrist wrist, BeltDrivenExtension belt, Intake intake, ArmPose goal) {
        this.mShoulder = shoulder;
        this.mWrist = wrist;
        this.goalPose = goal;
        this.mBelt = belt;
        this.original = new ArmPose(goalPose);

        addRequirements(shoulder, wrist, mBelt);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    public boolean mEnd = false;

    public ArmStateMachine(Shoulder shoulder, Wrist wrist, BeltDrivenExtension belt, Intake intake, ArmPose goal,
            boolean endOnPosition) {
        this.mShoulder = shoulder;
        this.mWrist = wrist;
        this.goalPose = goal;
        this.mBelt = belt;
        this.original = new ArmPose(goalPose);
        mEnd = endOnPosition;
        addRequirements(shoulder, wrist, mBelt);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    public ArmStateMachine(Shoulder shoulder, Wrist wrist, BeltDrivenExtension belt, Intake intake, Pose goal) {
        this.mShoulder = shoulder;
        this.mWrist = wrist;
        this.mBelt = belt;
        this.goal = goal;

        addRequirements(shoulder, wrist, mBelt);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    private boolean flag = false;
    private boolean zeroFlag = false;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        atPose = false;
        flag = false;
        ArmPose o = original;
        if (o == null) {
            o = Constants.Pose.getArmPose(goal);
        }
        if (lastPose != null) {
            if (lastPose.equals(Constants.ArmPoses.BACKWARDS_HIGH_CUBE)
                    && (o.shoulder.getDegrees()<120.)) {
                flag = true;
            }

            
        }

        


        lastPose = new ArmPose(o);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (goal != null) {

            this.goalPose = Constants.Pose.getArmPose(goal);
            this.original = new ArmPose(goalPose);

        }

        goalPose = new ArmPose(original);
        curPose = new ArmPose(Rotation2d.fromDegrees(this.mShoulder.getShoulderAbsolutePosition()), mBelt.getLength(),
                Rotation2d.fromRadians(mWrist.getWristPosition()));

        // If button is NOT pressed then...

        if (!mBelt.beltZero) {

            mBelt.disableReverseSoftLimit();

            if (!mBelt.isLimitSwitchPressed()) {
                mBelt.setSpeed(Constants.BeltDrive.HOMING_SPEED);
            } else {
                mBelt.setSpeed(0.0);
                mBelt.enableReverseSoftLimit();
            }

            return;
        }

        deltaShoulder = DaisyMath
                .boundAngleNegPiToPiRadians(
                        goalPose.shoulder.getRadians() - curPose.shoulder.getRadians());
        deltaWrist = DaisyMath
                .boundAngleNegPiToPiRadians(goalPose.wrist.getRadians() - curPose.wrist.getRadians());
        deltaLength = goalPose.belt - curPose.belt;

        beltState = ArmState.MOVING;
        shoulderState = ArmState.MOVING;
        wristState = ArmState.MOVING;
        if (flag) {
            beltState = ArmState.MOVING;
            shoulderState = ArmState.MOVING;
            if (Math.abs(deltaShoulder) < Math.toRadians(105.)) {
                wristState = ArmState.MOVING;
            } else {
                wristState = ArmState.WAITING_TO_MOVE;
            }
        } else {
            // yada yada yada, see diagram on drive
            if (Math.abs(deltaShoulder) > Constants.StateMachine.SHOULDER_STOWING_THRESHOLD) {
                beltState = ArmState.STOW;

                // wristState = ArmState.STOW;
                shoulderState = ArmState.MOVING;
                if (goalPose.isEndEffectorNearBumper() && curPose.beta.getRadians() <= 0.0) {
                    // wristState = ArmState.AVOID_GROUND;

                }
            } else if (Math.abs(deltaShoulder) < Constants.StateMachine.SHOULDER_APPROACHING_THRESHOLD) {
                if (Math.abs(deltaWrist) > Constants.StateMachine.WRIST_THETA_STOWING_THRESHOLD) {
                    wristState = ArmState.MOVING;
                    beltState = ArmState.STOW;
                } else if (Math.abs(deltaWrist) < Constants.StateMachine.WRIST_THETA_APPROACHING_THRESHOLD) {
                    beltState = ArmState.MOVING;
                    wristState = ArmState.MOVING;
                } else {
                    wristState = ArmState.MOVING;
                    beltState = ArmState.WAITING_TO_MOVE;
                }

            } else {
                beltState = ArmState.WAITING_TO_MOVE;
                wristState = ArmState.WAITING_TO_MOVE;
                shoulderState = ArmState.MOVING;
            }
        }

        if (curPose.shoulder.getDegrees() < -30.
                && (goalPose.shoulder.getDegrees() - curPose.shoulder.getDegrees() > 15.0)) {
            wristState = ArmState.WAITING_TO_MOVE;
        }
        if (curPose.wrist.getDegrees()<0 && curPose.shoulder.getDegrees() < -20. && (goalPose.shoulder.getDegrees() - curPose.shoulder.getDegrees() < -10.0)) {
            wristState = ArmState.STOW;
            shoulderState = ArmState.WAITING_TO_MOVE;

        }
    

        // this logic works
        if (shoulderState != ArmState.WAITING_TO_MOVE)
            this.mShoulder.setShoulderPosition(goalPose.shoulder.getDegrees());

      
        if(!zeroFlag&&goalPose.equals(Constants.ArmPoses.SHELF)) {
            mBelt.disableReverseSoftLimit();

            if (!mBelt.isLimitSwitchPressed()) {
                mBelt.setSpeed(Constants.BeltDrive.HOMING_SPEED);
            } else {
                mBelt.setSpeed(0.0);zeroFlag=true;
                mBelt.enableReverseSoftLimit();
            }
        }
        else if (beltState == ArmState.STOW) {
            goalPose.belt = Constants.BeltDrive.STOWED_POSITION;
            this.mBelt.setPosition(goalPose.belt);
        } else if (beltState == ArmState.WAITING_TO_MOVE) {
            this.mBelt.setPosition(Math.min(curPose.belt, goalPose.belt));
        } else {
            this.mBelt.setPosition(goalPose.belt);
        }

        if (wristState == ArmState.AVOID_GROUND) {
            goalPose.wrist = Rotation2d.fromDegrees(-goalPose.shoulder.getDegrees());
        } else if (wristState == ArmState.STOW) {
            goalPose.wrist = Rotation2d.fromDegrees(Constants.Wrist.STOW_ANGLE); 
        } else if (wristState == ArmState.WAITING_TO_MOVE) {
            goalPose.wrist = curPose.wrist;

        }

        this.mWrist.setPosition(goalPose, curPose);

        if (Math.abs(deltaShoulder) < Math.toRadians(Constants.Shoulder.SHOULDER_ANGLE_TOLERANCE)) {
            shoulderState = ArmState.FINISHED;
        }
        if (Math.abs(deltaWrist) < Constants.Wrist.ANGLE_TOLERANCE) {
            wristState = ArmState.FINISHED;
        }
        if (Math.abs(deltaLength) < Constants.BeltDrive.POSITION_TOLERANCE) {
            beltState = ArmState.FINISHED;
        }



        if (wristState == ArmState.FINISHED &&
                shoulderState == ArmState.FINISHED &&
                beltState == ArmState.FINISHED) {
            atPose = true;

        } else
            atPose = false;

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mEnd && atPose;
    }
}
