// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.TreeMap;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.revrobotics.Rev2mDistanceSensor.Port;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lib.util.COTSFalconSwerveConstants;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.StateMachine.ArmPose;
import frc.robot.utilities.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean FMSDETACHED = !DriverStation.isFMSAttached();
    public static final int PDH_PORT = 1;
    public static final double TRIGGER_THRESHOLD = 0.05;
    public static final double stickDeadband = 0.1;
    public static final Pose2d STARTING_POSE = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(6).get().toPose2d()
            .plus(new Transform2d(new Translation2d(Constants.Field.TAG_TO_FRONT, 0.0), new Rotation2d()));

    public static class CATAPULT {

        public static final int LAUNCHER_PORT = 1;

    }

    public static class ArmGeometry {
        public static final double MAST_X_DISTANCE_FROM_ROBOT_CENTER = -11.0; // inches
        public static final double SHOULDER_HEIGHT_FROM_GROUND = 25.0; // inches
        public static final double BELT_OUTER_STAGE = 25.0; // inches
        public static final double WRIST_LENGTH = 14.0; // inches

        public static final double BUMPER_HEIGHT = 7.5; // inches
        public static final double END_EFFECTOR_HEIGHT_FROM_FLOOR = 1.0; // inches
        public static final double MAST_TO_FRONT_BUMPER_DISTANCE = 26.0; // inches
        public static final double WRIST_SAFETY_BUMPER_THRESH = 12.; // inches

        public static final double FORWARD_EXTENSION_LIMIT = 26.0 + 48.0; // inches
        public static final double REAR_EXTENSION_LIMIT = 4.0 + 48.0; // inches
        public static final double CEILING_EXTENSION_LIMIT = 78.0; // inches

    }

    public static class ArmPoses {
        public static final ArmPose INTAKE_POSITION_CONE = new ArmPose(Rotation2d.fromDegrees(-41.), 0.25,
                Rotation2d.fromDegrees(42.0));
        public static final ArmPose INTAKE_POSITION_CUBE = new ArmPose(Rotation2d.fromDegrees(-41.), 0.25,
                Rotation2d.fromDegrees(42.0));

        public static final ArmPose BACKWARDS_HIGH_CONE = new ArmPose(Rotation2d.fromDegrees(131.13 - 3.0), 12.12,
                Rotation2d.fromDegrees(92.22 - 5.0));

        public static final ArmPose BACKWARDS_HIGH_CONE_AUTO = new ArmPose(Rotation2d.fromDegrees(131.13 - 3.0), 12.12,
                Rotation2d.fromDegrees(92.22 - 5.0));
        public static final ArmPose BACKWARDS_HIGH_CUBE = new ArmPose(Rotation2d.fromDegrees(134.), 0.5,
                Rotation2d.fromDegrees(56.0 - 5.0));

        public static final ArmPose BACKWARDS_MID_CONE = new ArmPose(Rotation2d.fromDegrees(122.), 0.5,
                Rotation2d.fromDegrees(130.8 - 5.0));
        public static final ArmPose BACKWARDS_MID_CUBE = new ArmPose(Rotation2d.fromDegrees(117.), 0.5,
                Rotation2d.fromDegrees(121.0 - 5.0));

        public static final ArmPose LOW_SCORE = new ArmPose(Rotation2d.fromDegrees(-25.), 0.25,
                Rotation2d.fromDegrees(33. + 5.0));

        public static final ArmPose AUTON_HIGH = new ArmPose(Rotation2d.fromDegrees(33.), 0.25,
                Rotation2d.fromDegrees(157.0 - 5.0));

        public static final ArmPose STOW = new ArmPose(Rotation2d.fromDegrees(-36.), 0.25,
                Rotation2d.fromDegrees(Wrist.STOW_ANGLE));
        public static final ArmPose ABSOLUTE_STOW = new ArmPose(Rotation2d.fromDegrees(-45.), 0.1,
                Rotation2d.fromDegrees(170.0));
        public static final ArmPose SHELF = new ArmPose(Rotation2d.fromDegrees(41.5), 0.1,
                Rotation2d.fromDegrees(-33.7 + 7.0));

        public static final ArmPose DROP = new ArmPose(Rotation2d.fromDegrees(-12.), 0.5,
                Rotation2d.fromDegrees(60.8 + 5.0));

        public static final ArmPose FORWARD_MID_CUBE = new ArmPose(Rotation2d.fromDegrees(13.0), 0.50,
                Rotation2d.fromDegrees(0.0 + 5.0));

    }

    public static enum Pose {
        INTAKE, BACKWARDS_HIGH, BACKWARDS_MID, FORWARD_MID;

        public static ArmPose getArmPose(Pose desiredPose) {
            if (desiredPose == INTAKE) {
                if (frc.robot.subsystems.Intake.coneMode) {
                    return ArmPoses.INTAKE_POSITION_CONE;
                }
                return ArmPoses.INTAKE_POSITION_CUBE;
            }
            if (desiredPose == BACKWARDS_HIGH) {
                if (frc.robot.subsystems.Intake.coneMode) {
                    return ArmPoses.BACKWARDS_HIGH_CONE;
                }
                return ArmPoses.BACKWARDS_HIGH_CUBE;
            }

            if (desiredPose == BACKWARDS_MID) {
                if (frc.robot.subsystems.Intake.coneMode) {
                    return ArmPoses.BACKWARDS_MID_CONE;
                }
                return ArmPoses.BACKWARDS_MID_CUBE;
            }
            if (desiredPose == FORWARD_MID) {

                return null;
            }
            return null;
        }
    }

    // Thank you, 2539!
    public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);

        /* X Placement constants from 6328 */
        public static final double outerX = Units.inchesToMeters(54.25 + 4.5 + 7. + 6.0 - 6.0);
        public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube
                                                                                        // nodes
        public static final double midX = outerX - Units.inchesToMeters(22.75);
        public static final double highX = outerX - Units.inchesToMeters(39.75);

        /* Z Placement constants from 6328 */
        public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
        public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
        public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
        public static final double highConeZ = Units.inchesToMeters(46.0);
        public static final double midConeZ = Units.inchesToMeters(34.0);

        public static class PlacementLocation {
            public Pose2d robotPlacementPose;
            public boolean isCone;

            public PlacementLocation(Pose2d poseAlignedWithEdge, double lengthOfRobotWithBumpers, boolean isCone) {
                var transformHybridToRobot = new Transform2d(new Translation2d(lengthOfRobotWithBumpers / 2, 0),
                        Rotation2d.fromDegrees(0));
                robotPlacementPose = poseAlignedWithEdge.transformBy(transformHybridToRobot);
                this.isCone = isCone;
            }

            public Pose3d getPose() {
                return new Pose3d(Field.TAG_TO_FRONT, robotPlacementPose.getY(), 0, new Rotation3d());
            }

            public Pose3d getHybridPose() {
                return new Pose3d(lowX, robotPlacementPose.getY(), 0, new Rotation3d());
            }

            public Pose3d getMidPose() {
                return new Pose3d(midX, robotPlacementPose.getY(), isCone ? midConeZ : midCubeZ, new Rotation3d());
            }

            public Pose3d getHighPose() {
                return new Pose3d(highX, robotPlacementPose.getY(), isCone ? highConeZ : highCubeZ, new Rotation3d());
            }
        }

        public static final int numberOfNodeRows = 9;
        public static final double separationBetweenNodeRows = Units.inchesToMeters(22.0);
        public static final Pose2d firstPlacingPose = new Pose2d(outerX, Units.inchesToMeters(20.19), new Rotation2d());

        public static final boolean[] isCone = new boolean[] { true, false, true, true, false, true, true, false,
                true };

        // Store the locations we will score from on the field (for automatic placement)
        public static final PlacementLocation[] placingPoses = new PlacementLocation[numberOfNodeRows];

        static {
            for (int i = 0; i < numberOfNodeRows; i++) {
                placingPoses[i] = new PlacementLocation(
                        new Pose2d(
                                firstPlacingPose.getX(),
                                firstPlacingPose.getY() + i * separationBetweenNodeRows,
                                new Rotation2d()),
                        Swerve.lengthWithBumpers,
                        isCone[i]);
            }
        }

        private static TreeMap<Double, PlacementLocation> locationMap = new TreeMap<>();

        static {
            for (PlacementLocation placementLocation : placingPoses) {
                if (!placementLocation.isCone)
                    locationMap.put(placementLocation.robotPlacementPose.getY(), placementLocation);
            }
        }

        /**
         * Finds the game piece placement area closest to the robot.
         * 
         * @param robotPose
         * @return The nearest placement location
         */
        public static PlacementLocation getNearestPlacementLocation(Pose2d robotPose) {
            double target = DriverStation.getAlliance() == Alliance.Red ? fieldWidth - robotPose.getY()
                    : robotPose.getY();
            Double lowerYValue = locationMap.floorKey(target);
            Double upperYValue = locationMap.ceilingKey(target);

            // Account for the pose being below or above the range
            if (lowerYValue == null)
                return flipPlacementLocation(locationMap.get(upperYValue));
            else if (upperYValue == null)
                return flipPlacementLocation(locationMap.get(lowerYValue));

            boolean isLowerCloser = Math.abs(target - lowerYValue) < Math.abs(target - upperYValue);

            var finalPlacementLocation = isLowerCloser ? locationMap.get(lowerYValue) : locationMap.get(upperYValue);

            return flipPlacementLocation(finalPlacementLocation);
        }

        public static PlacementLocation flipPlacementLocation(PlacementLocation placementLocation) {
            PlacementLocation resultPlacementLocation = placementLocation;

            if (DriverStation.getAlliance() == Alliance.Red) {
                var pose = placementLocation.robotPlacementPose;
                resultPlacementLocation = new PlacementLocation(
                        new Pose2d(firstPlacingPose.getX(), fieldWidth - pose.getY(), pose.getRotation()),
                        Swerve.lengthWithBumpers,
                        placementLocation.isCone);
            }

            return resultPlacementLocation;
        }

        public static final List<AprilTag> aprilTags = List.of(
                new AprilTag(
                        1,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        2,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        3,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(174.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        4,
                        new Pose3d(
                                Units.inchesToMeters(636.96),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        5,
                        new Pose3d(
                                Units.inchesToMeters(14.25),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d())),
                new AprilTag(
                        6,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(174.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())),
                new AprilTag(
                        7,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())),
                new AprilTag(
                        8,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())));

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(aprilTags, fieldLength,
                fieldWidth);

        public static void setAprilTagOrigin() {
            APRIL_TAG_FIELD_LAYOUT.setOrigin(
                    DriverStation.getAlliance() == Alliance.Red
                            ? OriginPosition.kRedAllianceWallRightSide
                            : OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    public static class StateMachine {

        public static final double SHOULDER_APPROACHING_THRESHOLD = Math.toRadians(30.);
        public static final double WRIST_THETA_STOWING_THRESHOLD = Math.toRadians(50.); 
        public static final double WRIST_THETA_APPROACHING_THRESHOLD = Math.toRadians(30.);
        public static final double SHOULDER_STOWING_THRESHOLD = Math.toRadians(40.);

    }

    public static class Shoulder {
        public static final int MASTER_MOTOR_PORT = 18;
        public static final int SLAVE_MOTOR_PORT = 19;

        public static final PIDGains SHOULDER_MOTOR_PID_GAINS = new PIDGains(0.5, 0.0, 0.0, 0.011, 0.0, 0.0, -1.0, 1.0,
                "Shoulder/Shoulder Gains");
        public static final double SENSOR_UNITS_TO_DEGREES = 0;
        public static final boolean kSensorPhase = false;
        public static final TalonFXInvertType kMotorInvert = null;

        public static final double SHOULER_SPEED = -1.0;
        public static final double SHOULDER_ANGLE_TOLERANCE = 3.0; // Degrees

        public static int kPIDLoopIdx = 0;
        public static int kTimeoutMs = 20;

        public static final int CANCODER_PORT = 5;
        public static final double CANCODER_ABSOLUTE_POSITION_CONVERSION_FACTOR = 1.0;
        public static final double kS = 0.01;
        public static final double kv = 0.01;
        public static final double SHOULDER_FF_KS = 0.0;
        public static final double SHOULDER_FF_KG = 0.0;
        // public static final double SHOULDER_FF_KV = 1.0;
        public static final double SHOULDER_FF_KV = 0.0;

        public static final double SHOULDER_TALON_POSITION_FACTOR = 2048.0 * 234 / 360;// 0.75;//0.7125;//0.782;//1.25 *
                                                                                       // 20.0 / 12.0;/*
        // * 1.0 / 583.333 * 180.0 / Math.PI *
        // */ /* 2048.0 * (459.5 / 360); */

        // public static final double SHOULDER_TALON_POSITION_FACTOR = 4.667 * 25.0;
        public static double CANCODER_ABSOLUTE_POSITION_OFFSET;

        static {
            if (Robot.blueRobot) {
                CANCODER_ABSOLUTE_POSITION_OFFSET = -4.9 - 3.2 - 59.85;
            } else {
                CANCODER_ABSOLUTE_POSITION_OFFSET = 31.0;

            }
        }
        // absolute encoder reading for where the
        // climber arm should idealy be in
        // starting config
        public static final double CANCODER_TO_OUTPUT_STAGE = 1.0; // For every 1 rotations, the shoulder will
                                                                   // rotate once
        public static final double SHOULDER_STARTING_POSITION = -35.0; // 0 degrees is horizontal, clockwise (looking
                                                                       // from the side with intake pointing left) is
                                                                       // positive, so starting pos will be negative
        public static final double UPPER_SHOULDER_LIMIT = 120.;
        public static final double LOWER_SHOULDER_LIMIT = -30.;
        public static final double MAX_OUTPUT = 1.;
        public static final double ARM_SPEED = 16000 * 2.;
        public static final double ARM_ACCEL = 16000;

        public static final double angleToTalon = 2048.0 * 234.0 / 360.0;
        public static final double talonToAngle = 1.0 / angleToTalon;

    }

    public static class Intake {

        public static final int INTAKE_MOTOR_PORT = 25;
        public static final int INTAKE_MOTOR_CURRENT_LIMIT = 20;
        public static final double RUN_SPEED = 1.0;
        public static final double REVERSE_SPEED = -0.75;
        public static final int GAME_PIECE_ACQUIRED_COUNT_CONE = 7;
        public static final int GAME_PIECE_ACQUIRED_COUNT_CUBE = 1;
        public static final int BEAM_BREAK_PORT = 2;

    }

    public static class Wrist {
        public static final int WRIST_MOTOR_PORT = 26;
        public static final int WRIST_ENCODER_PORT = 0;
        public static final int SOLENOID_PORT = 0;

        public static final double MAX_OUTPUT = 0.85;
        public static final int CURRENT_LIMIT = 30;

        public static final double WRIST_POSITION_FILTER_ALPHA = 0.075; // Takes only 10% of current command point to be
                                                                        // applied at a time.

        public static final double SERVO_LOCK_POSITION = 120.0;
        public static final double SERVO_UNLOCK_POSITION = 160.0;

        public static final double WRIST_POSITION_CONVERSION_FACTOR = 1.0 / 50.0 * Math.PI * 2.0; // 62.5 units = 1
                                                                                                  // revolution, used to
        // convert motor units, so motorUnits
        // * conversionFactor = radians
        public static final double ANGLE_TOLERANCE = Math.toRadians(9.0); // radians
        public static final double WRIST_ZERO_OFFSET; // radians to shift absolute encoder reading
                                                      // so that when wrist is parallel with arm,
                                                      // it reads zero
        static {
            if (Robot.blueRobot) {
                WRIST_ZERO_OFFSET = Math.toRadians(-135.);
            } else {
                WRIST_ZERO_OFFSET = Math.toRadians(-54.);

            }
        }

        public static final PIDGains WRIST_PID = new PIDGains(1.0, 0.0, 0.0, 0.0, 0., 0.0,
                -Constants.Wrist.MAX_OUTPUT, Constants.Wrist.MAX_OUTPUT,
                "Wrist/Wrist Gains");

        public static final double MAX_VELOCITY = 30.;// Math.PI; // radians/sec
        public static final double MAX_ACCELERATION = Math.PI / 10; // radians/sec^2
        public static final int SERVO_UNLOCK_COUNT = 3;
        public static final double STOW_ANGLE = 150.;

        public static final double WRIST_SOFT_MIN_LIMIT = Math.toRadians(-63.0);
        public static final double WRIST_SOFT_MAX_LIMIT = Math.toRadians(120.0);
        public static final double THROUGH_BORE_CONV_FACTOR = 2. * Math.PI;
    }

    public static class BeltDrive {
        public static final int BELT_MOTOR_PORT = 20;
        // public static final int ENCODER_PORT = 3;

        public static final double MAX_OUTPUT = 1.;
        public static final int CURRENT_LIMIT = 30;

        public static final double BELT_POSITION_FILTER_ALPHA = 0.1; // Takes only 10% of current command point to be
                                                                     // applied at a time.

        public static final double BELT_POSITION_CONVERSION_FACTOR = 1.0 / 17.5 * Math.PI * 0.91; // 10 units = 1
                                                                                                  // revolution,

        public static final double POSITION_TOLERANCE = 0.25; // inches
        public static final double POSITION_ZERO_OFFSET; // inches
        static {
            if (Robot.blueRobot) {
                POSITION_ZERO_OFFSET = 0;
            } else {
                POSITION_ZERO_OFFSET = 0;
            }
        }
        public static final PIDGains BELT_PID = new PIDGains(0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                -Constants.BeltDrive.MAX_OUTPUT, Constants.BeltDrive.MAX_OUTPUT,
                "Belt Drive/Belt Gains");
        public static final double STOWED_POSITION = 0.1;

        public static final double EXTENSION_SOFT_MIN_LIMIT = 0.0;
        public static final double EXTENSION_SOFT_MAX_LIMIT = 50.0;
        public static final int LIMIT_SWITCH_PORT = 1;

        public static final double HOMING_SPEED = -0.25;
        public static final double ZERO_EPSILON = 0.05;

        public static final boolean INVERT_LIMIT_SWITCH;

        public static final double MOVE_SLOW_THRESHOLD = 1.0;
        static {
            if (Robot.blueRobot)
                INVERT_LIMIT_SWITCH = false;
            else
                INVERT_LIMIT_SWITCH = true;
        }
    }

    public static class Drivebase {
        // Max drive base voltage, can be lowered for testing
        public static final double MAX_VOLTAGE = 12.0;

        public static final double DRIVEBASE_TRACKWIDTH_METERS = Units.inchesToMeters(17.75);
        public static final double DRIVEBASE_WHEELBASE_METERS = Units.inchesToMeters(24.75);

        public static final int FRONT_LEFT_DRIVE_PORT = 10;
        public static final int FRONT_LEFT_TURN_PORT = 14;
        public static final int FRONT_LEFT_TURN_ENCODER_PORT = 6;

        public static final int BACK_LEFT_DRIVE_PORT = 11;
        public static final int BACK_LEFT_TURN_PORT = 15;
        public static final int BACK_LEFT_TURN_ENCODER_PORT = 7;

        public static final int BACK_RIGHT_DRIVE_PORT = 12;
        public static final int BACK_RIGHT_TURN_PORT = 16;
        public static final int BACK_RIGHT_TURN_ENCODER_PORT = 8;

        public static final int FRONT_RIGHT_DRIVE_PORT = 13;
        public static final int FRONT_RIGHT_TURN_PORT = 17;
        public static final int FRONT_RIGHT_TURN_ENCODER_PORT = 9;

        // public static final PIDGains THETA_CONTROLLER_GAINS = new PIDGains(0.0);
        public static final PIDGains THETA_CONTROLLER_GAINS = new PIDGains(3.0, 0.0, 0.1);// 0.125);// 41.543, 0.0,
                                                                                          // 2.6885);

        public static final PIDGains TURN_BODY_PID_GAINS = new PIDGains(0.1, 0.0, 0.0);

        // public static final PIDGains DRIVE_PID_GAINS = new PIDGains(0.0001);
        public static final PIDGains DRIVE_PID_GAINS = new PIDGains(0.0, 0.0, 0.0); // 0.0125);//3.0 / 12.0);//0.000625
                                                                                    // / 12.0);

        public static final PIDGains TURN_PID_GAINS = new PIDGains(0.25, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
                "turn pid");

        public static final double MAX_ANGULAR_VELOCITY = 100.0 / 2.0;// 40;
        public static final double MAX_ANGULAR_ACCELERATION = 50.0 / 2.0;// 20.0;

        public static final double kV = 2.3599 / 12.0;/// * 0.69433 */ 2.5 / 12.0;// 0.7248;
        public static final double kS = 0.71695 / 12.0;// 0.74871 / 12.0;// 0.54214;
        public static final double kA = 0.42457 / 12.0;// 0.10884;/// 0.09744;

        public static final double MODULE_DRIVE_DISTANCE_CONVERSION_FACTOR = (1.0 * 10 * Math.PI
                * Units.inchesToMeters(3.85)) / (2048.0 * 6.75);

        public static final double MODULE_DRIVE_VELOCITY_CONVERSION_FACTOR = (1.0 * 10 * Math.PI
                * Units.inchesToMeters(3.85)) / (2048.0 * 6.75);

        public static final double DISTANCE_PER_REVOLUTION = 1.0 / 6.75 / 60 * 2.0 * Math.PI * (4.7 / 2.0) * 0.0254; // m/s
        // Module Gearing: 6.75:1 (14->50, 27->17, 15->45)
        // Wheel Diameter: 4"
        // C = 2*pi*r
        // inches to meter: 0.0254

        // BLUE
        // public static final double BLUE_FRONT_LEFT_ANGLE_OFFSET = 0.0; // + 180.0;
        // public static final double BLUE_FRONT_RIGHT_ANGLE_OFFSET = 121; // + 180.0;
        // public static final double BLUE_BACK_LEFT_ANGLE_OFFSET = -87.54;// + 180.0;
        // public static final double BLUE_BACK_RIGHT_ANGLE_OFFSET = 169.45;// + 180.0;

        public static final double BLUE_FRONT_LEFT_ANGLE_OFFSET = 158.6; // + 180.0;
        public static final double BLUE_FRONT_RIGHT_ANGLE_OFFSET = -58.10; // + 180.0;
        public static final double BLUE_BACK_LEFT_ANGLE_OFFSET = 91.76;// + 180.0;
        public static final double BLUE_BACK_RIGHT_ANGLE_OFFSET = -10.99;// + 180.0;

        // SILVER
        public static final double SILVER_FRONT_LEFT_ANGLE_OFFSET = 156.894521; // 0.0;// + 180.0;
        public static final double SILVER_FRONT_RIGHT_ANGLE_OFFSET = 33.664198;// 0.0;// + 180.0;
        public static final double SILVER_BACK_LEFT_ANGLE_OFFSET = 91.910545;// 0.0;// + 180;
        public static final double SILVER_BACK_RIGHT_ANGLE_OFFSET = 349.972380;// 0.0;// + 180;

        // Use to find the new angle offset
        // public static final double FRONT_LEFT_ANGLE_OFFSET = 0;
        // public static final double FRONT_RIGHT_ANGLE_OFFSET = 0;
        // public static final double BACK_LEFT_ANGLE_OFFSET = 0;
        // public static final double BACK_RIGHT_ANGLE_OFFSET = 0;

        public static final PIDGains AUTO_PID_GAINS = new PIDGains(0.5, 0.0, 0.0);
        public static final double VISION_ANGLE_TOLERANCE = 3.0;

        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 4;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI * 8;
        public static final int MAGNETIC_SENSOR_PORT = 7;
        public static final double JUKE_DISTANCE = 120.0; // Inches

        public static final Constraints kThetaControllerConstraints = new Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final double TURN_SLEW_RATE = 0.5;

        public static final double VISION_KF = ((MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 90.0));// 1.0 / (360.0 /
                                                                                               // MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

        public static final PIDGains ROTATE_PID_GAINS = new PIDGains(0.1, 0.0, 0.0, VISION_KF);

        public static final PIDGains PREDICTIVE_PID_GAINS = new PIDGains(0.0, 0.0, 0.0,
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 45.0);

        public static final double SPEED_REDUCTION_FACTOR = 3.0;

        public static final int IMU_ID = 0;

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(5.0, 3.0);

    }

    public static class ControllerInputs {

        public static final double DEADBAND = 0.05;

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int PROGRAMMER_CONTROLLER_PORT = 2;
    }

    public static class Field {
        // public static final double TAG_TO_SIDE = 0.582;
        public static final double TAG_TO_SIDE = Units.inchesToMeters(22.0);

        public static final double TAG_TO_FRONT = Units.inchesToMeters(30.0); 

        public static final Translation2d BLUE_BOTTOM_PIECE_FIELD_LOCATION = new Translation2d(6.67,
                0.91);

        public static final Translation2d RED_BOTTOM_PIECE_FIELD_LOCATION = new Translation2d(6.67,
                8.0 - BLUE_BOTTOM_PIECE_FIELD_LOCATION.getY());

        public static final Translation2d BLUE_MID_TOP_PIECE_FIELD_LOCATION = new Translation2d(6.67, 3.37);
        public static final Translation2d RED_MID_TOP_PIECE_FIELD_LOCATION = new Translation2d(6.67,
                8.0 - BLUE_MID_TOP_PIECE_FIELD_LOCATION.getY());
        public static final Translation2d BLUE_MID_BOTTOM_PIECE_FIELD_LOCATION = new Translation2d(6.67, 2.16);
        public static final Translation2d RED_MID_BOTTOM_PIECE_FIELD_LOCATION = new Translation2d(6.67,
                8.0 - BLUE_MID_BOTTOM_PIECE_FIELD_LOCATION.getY());
    }

    public static class LED {

        public static final int LED_PORT = 10;

        public static final int BOTTOM_START_INDEX = 7;
        public static final int TOP_START_INDEX = 16;
        public static final int FIRING_START_INDEX = 23;

        public static final int CANDLE_START_INDEX = 0;

        public static final int LEFT_LED_LENGTH = 26;
        public static final int RIGHT_LED_LENGTH = 26;
        public static final int CANDLE_LED_LENGTH = 8;

        public static final int LED_STRING_LENGTH = LEFT_LED_LENGTH + RIGHT_LED_LENGTH + CANDLE_LED_LENGTH;

        public static final Color CONE_COLOR = Color.kYellow;
        public static final Color CUBE_COLOR = Color.kPurple;
        public static final Color ALIGNED_COLOR = Color.kGreen;
        public static final Color UNALIGNED_COLOR = Color.kRed;
        public static final Color INTOOKEN_COLOR = Color.kOrange;
        public static final Color NOT_INTAKED = Color.kWhite;

        public static final Animation WIN_BM = new RainbowAnimation(1, 1.0, LED_STRING_LENGTH, false, 8);
        public static final int RIGHT_START_INDEX = 8;
        public static final int LEFT_START_INDEX = 34;

    }

    public static class Vision {
        public static final double LENGTH = 2.5;
        public static final double GOAL_HEIGHT = 7.458333333;
        public static final double LIMELIGHT_HEIGHT = 41.0 / 12.0; // height inches / inches per foot//1.833333;
        public static final double INITIAL_ANGLE = 35.0;
        public static final PIDGains VISION_PID_GAINS = new PIDGains(0.04, 0.0, 0.005);
        public static final double TX_FILTER = 0.75;

        public static final PIDGains DRIVE_VISION_PID_GAINS = new PIDGains(5.0 / 50.0);
        public static final String LIMELIGHT_NAME = "limelight";
        public static final int TX_THRESH = 10;
        public static final int TAPE_PIPELINE = 1;
        public static final int TAG_PIPELINE = 0;

    }

    public static final String CANIVORE_NAME;
    static {
        if (Robot.blueRobot) {
            CANIVORE_NAME = "CANivore";
        } else {
            CANIVORE_NAME = "CANIVORE 3";
        }
    }

    public static final class Swerve {
        public static final double lengthWithBumpers = Units.inchesToMeters(29.0);
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =
                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(17.73);
        public static final double wheelBase = Units.inchesToMeters(24.73);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 200.0 / 1000.;
        public static final double closedLoopRamp = 100.0 / 1000.;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = 0.71695 / 12.0; // (0.32 / 12);
        public static final double driveKV = 2.45 / 12.0; // (1.51 / 12);
        public static final double driveKA = 0.42457 / 12.0; // (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;

        public static final double SLOW_SPEED_MODIFIER = 0.25; // quarter speed
        public static final double SLOW_ROT_SPEED_MODIFIER = 0.25; // quarter speed

        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
        public static final double FORWARD_PIVOT_POINT = Units.inchesToMeters(40.0);
        public static final double VISION_KF = 0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset;

            static {
                if (Robot.blueRobot) {
                    angleOffset = Rotation2d.fromDegrees(14.1);
                } else {
                    angleOffset = Rotation2d.fromDegrees(159.6);
                }
            }
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 9;

            public static final Rotation2d angleOffset;

            static {
                if (Robot.blueRobot) {
                    angleOffset = Rotation2d.fromDegrees(83.5);

                } else {
                    angleOffset = Rotation2d.fromDegrees(306.);

                }
            }
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset;
            static {
                if (Robot.blueRobot) {
                    angleOffset = Rotation2d.fromDegrees(91.3);

                } else {
                    angleOffset = Rotation2d.fromDegrees(93.5);

                }
            }

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 16;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset;
            static {
                if (Robot.blueRobot) {
                    angleOffset = Rotation2d.fromDegrees(261.0);

                } else {
                    angleOffset = Rotation2d.fromDegrees(352.09);

                }
            }
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    
}
