// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.StateMachine;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmPose {
    public Rotation2d shoulder; // degrees
    public double belt; // inches
    public Rotation2d wrist; // degrees
    public Rotation2d beta; // degrees

    public double d1 = Constants.ArmGeometry.SHOULDER_HEIGHT_FROM_GROUND;
    public double d2 = Constants.ArmGeometry.BELT_OUTER_STAGE;
    public double d3 = Constants.ArmGeometry.WRIST_LENGTH;

    private Pose3d beltJoint;
    private Pose3d wristJoint;
    private Pose3d endEffector;

    public ArmPose(Rotation2d shoulder, double belt, Rotation2d wrist) {
        this.shoulder = shoulder;
        this.belt = belt;
        this.wrist = wrist;
        this.beta = new Rotation2d(this.wrist.getRadians() + this.shoulder.getRadians());
        this.ComputeArmGeometry();
    }

    public ArmPose(ArmPose other) {
        this(other.shoulder, other.belt, other.wrist);
    }

    // default robot position
    public ArmPose() {
        this(new Rotation2d(), 0.0, new Rotation2d());
    }
    
    public void setFrom(ArmPose source) {
        this.shoulder = source.shoulder;
        this.belt = source.belt;
        this.wrist = source.wrist;
        this.ComputeArmGeometry();
    }

    public void ComputeArmGeometry() {
        this.beltJoint = this.BeltStageLocation();
        this.wristJoint = this.WristLocation();
        this.endEffector = this.EndEffectorLocation();
    }

    /* Belt Drive Outer Stage Location */
    public Pose3d BeltStageLocation() {
        // Tip of arm outer stage (i.e. belt driven extension)
        double x_B = d2 * this.shoulder.getCos();
        double z_B = d1 + d2 * this.shoulder.getSin();

        return new Pose3d(x_B, 0.0, z_B, new Rotation3d());
    }

    /* Belt Drive Outer Stage Location */
    public Pose3d WristLocation() {
        // Location of wrist rotation joint
        double x_W = (d2 + this.belt) * this.shoulder.getCos();
        double z_W = d1 + (d2 + this.belt) * this.shoulder.getSin();

        return new Pose3d(x_W, 0.0, z_W, new Rotation3d());
    }

    /* End Effector Location */
    public Pose3d EndEffectorLocation() {
        // Equation to define the wrist angle relative to the ground
        // horizontal. This is helpful for when we want to define the end
        // effector to be held in a specific orientation for scoring, stow,
        // or pickup.
        Rotation2d beta = new Rotation2d(this.wrist.getRadians() + this.shoulder.getRadians());

        // Location of end effector tip
        double x_E = (d2 + this.belt) * this.shoulder.getCos() + d3 * beta.getCos();
        double z_E = d1 + (d2 + this.belt) * this.shoulder.getSin() + d3 * beta.getSin();

        return new Pose3d(x_E, 0.0, z_E, new Rotation3d());
    }

    public Pose3d getBeltJoint() {
        return beltJoint;
    }

    public Pose3d getWristJoint() {
        return wristJoint;
    }

    public Pose3d getEndEffectorLocation() {
        return endEffector;
    }

    /*
     * Checks to see if the arm pose is within bounds
     */

    public boolean isBeltJointAboveBumper() {
        return getBeltJoint().getZ() > Constants.ArmGeometry.BUMPER_HEIGHT;
    }

    public boolean isWristJointInFrontOfRobot() {
        return getWristJoint().getX() > Constants.ArmGeometry.MAST_TO_FRONT_BUMPER_DISTANCE;
    }

    public boolean isWristJointAboveBumper() {
        return getWristJoint().getZ() > Constants.ArmGeometry.BUMPER_HEIGHT;
    }

    public boolean isWristAboveFloor() {
        return getWristJoint().getZ() >= 0.0;
    }

    public boolean isWristBehindForwardExtensionLimit() {
        return getWristJoint().getX() < Constants.ArmGeometry.FORWARD_EXTENSION_LIMIT;
    }

    public boolean isEndEffectorBehindForwardExtensionLimit() {
        return getEndEffectorLocation().getX() < Constants.ArmGeometry.FORWARD_EXTENSION_LIMIT;
    }

    public boolean isEndEffectorNearBumper() {
        return getEndEffectorLocation().getZ() <= Constants.ArmGeometry.WRIST_SAFETY_BUMPER_THRESH;
    }



    public void bound() {
        if (shoulder.getDegrees() > Constants.Shoulder.UPPER_SHOULDER_LIMIT) {
            shoulder = Rotation2d.fromDegrees(Constants.Shoulder.UPPER_SHOULDER_LIMIT);
        } else if (shoulder.getDegrees() < Constants.Shoulder.LOWER_SHOULDER_LIMIT) {
            shoulder = Rotation2d.fromDegrees(Constants.Shoulder.LOWER_SHOULDER_LIMIT);
        }
        if (wrist.getRadians() > Constants.Wrist.WRIST_SOFT_MAX_LIMIT) {
            wrist = new Rotation2d(Constants.Wrist.WRIST_SOFT_MAX_LIMIT);
        } else if (wrist.getRadians()  < Constants.Wrist.WRIST_SOFT_MIN_LIMIT) {
            wrist = new Rotation2d(Constants.Wrist.WRIST_SOFT_MIN_LIMIT);

        }
        if (belt > Constants.BeltDrive.EXTENSION_SOFT_MAX_LIMIT) {
            belt = Constants.BeltDrive.EXTENSION_SOFT_MAX_LIMIT;
        }
        if (belt < Constants.BeltDrive.EXTENSION_SOFT_MIN_LIMIT) {
            belt = Constants.BeltDrive.EXTENSION_SOFT_MIN_LIMIT;
        }
        

       
    }

    @Override
    public String toString() {
        return "ArmState{" +
                "Shoulder=" + shoulder +
                ", belt=" + belt +
                ", wrist=" + wrist +
                '}';
    }

    public boolean equals(ArmPose other) {
        return other.shoulder.equals(this.shoulder) && other.wrist.equals(this.wrist) && other.belt == this.belt;
    }

    public Double[] asVector() {
        return new Double[] { shoulder.getDegrees(), belt, wrist.getDegrees() };
    }

}
