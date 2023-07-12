// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.TopTwoCycle;
import frc.robot.Autonomous.TopTwoHalfBalance;
import frc.robot.Autonomous.TopTwoBalance;
import frc.robot.Autonomous.ChargingStationCommands.DriveBackwardAndLevel;
import frc.robot.Autonomous.ChargingStationCommands.DriveForwardAndLevel;
import frc.robot.Constants.ArmPoses;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Pose;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.robot.Autonomous.BottomTwoCycleTag;
import frc.robot.Autonomous.BottomTwoCycleTagBalance;
import frc.robot.Autonomous.BottomTwoHalfCycle;
import frc.robot.Autonomous.DriveOverStationBalance;
import frc.robot.Autonomous.DriveOverStationBalanceCone;
import frc.robot.Autonomous.DriveOverStationPickupBalance;
import frc.robot.Autonomous.DriveOverStationPickupBalanceFling;
import frc.robot.Autonomous.ScoreAndBalanceForward;
import frc.robot.Autonomous.TopThreeBack;
import frc.robot.commands.BeltDrivenExtension.HomeBeltToZero;
import frc.robot.commands.BeltDrivenExtension.SetBeltSpeed;
import frc.robot.commands.Drive.DiamondFormation;
import frc.robot.commands.Drive.DriveToPositionAbsolute;
import frc.robot.commands.Drive.ResetAll;
import frc.robot.commands.Drive.SetSwerveHeading;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Intake.IntakePiece;
import frc.robot.commands.Intake.OuttakePiece;
import frc.robot.commands.LED.RunLED;
import frc.robot.commands.Shoulder.SetShoulderSpeed;
import frc.robot.commands.Wrist.SetWristSpeed;
import frc.robot.subsystems.BeltDrivenExtension;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateMachine.ArmStateMachine;
import frc.robot.subsystems.StateMachine.Fling;
import frc.robot.subsystems.StateMachine.TurnStateMachineOff;
import frc.robot.subsystems.tracking.LimelightInterface;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // Drivebase mDrivebase = Drivebase.getInstance();
        LimelightInterface mLimelightInterface = LimelightInterface.getInstance();
        LED mLED = LED.getInstance();
        Shoulder mShoulder = Shoulder.getInstance();
        Intake mIntake = Intake.getInstance();
        Catapult mCatapult = Catapult.getInstance();
        Wrist mWrist = Wrist.getInstance();
        BeltDrivenExtension mBelt = BeltDrivenExtension.getInstance();

        private final XboxController mDriverController = new XboxController(
                        Constants.ControllerInputs.DRIVER_CONTROLLER_PORT);
        private final XboxController mOperatorController = new XboxController(
                        Constants.ControllerInputs.OPERATOR_CONTROLLER_PORT);
        private final XboxController mProgrammingController = new XboxController(
                        Constants.ControllerInputs.PROGRAMMER_CONTROLLER_PORT);

        /* Controllers */
        private final Joystick driver = new Joystick(0);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;

        /* Driver Buttons */
        private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

        /* Subsystems */
        private final Swerve s_Swerve = Swerve.getInstance();

      

        private SendableChooser<Command> mAutonomousChooser;

        public boolean isAligningToTag() {
                return false;
        }

        public static Supplier<Pose2d> getScoringLocation(Swerve swerve, double offsetY) {
                return () -> {
                        PlacementLocation targetLocation = FieldConstants
                                        .getNearestPlacementLocation(swerve.getPoseAbsolute());

                        return targetLocation.robotPlacementPose
                                        .plus(new Transform2d(new Translation2d(0., offsetY),
                                                        new Rotation2d()));
                };
        }

        public static Supplier<Pose2d> getScoringLocation(Swerve swerve, double offsetY, double offsetX) {
                return () -> {
                        PlacementLocation targetLocation = FieldConstants
                                        .getNearestPlacementLocation(swerve.getPoseAbsolute());
                        return targetLocation.robotPlacementPose
                                        .plus(new Transform2d(
                                                        new Translation2d(offsetX, offsetY), //
                                                        new Rotation2d()));
                };
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         * 
         * @throws FileNotFoundException
         */
        public RobotContainer() throws FileNotFoundException {

                // Real ones
                mAutonomousChooser = new SendableChooser<Command>();
                mAutonomousChooser.addOption("Drive Forward and Level", new DriveForwardAndLevel(s_Swerve));
                mAutonomousChooser.addOption("Drive Over Station, Pickup, and Balance Blue",
                                new DriveOverStationPickupBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, true, false));
                mAutonomousChooser.addOption("Scoare and Balance Forwards ",
                                new ScoreAndBalanceForward(s_Swerve, mShoulder, mWrist, mBelt, mIntake));
                mAutonomousChooser.addOption("Drive Over Station, Pickup, and Balance Red",
                                new DriveOverStationPickupBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, false, false));
                mAutonomousChooser.addOption("Drive Over Station, Pickup, and Balance Blue-Fling",
                                new DriveOverStationPickupBalanceFling(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, true));
                mAutonomousChooser.addOption("Drive Over Station, Pickup, and Balance Red-Fling",
                                new DriveOverStationPickupBalanceFling(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, false));
                mAutonomousChooser.addOption("Drive Over Station Balance Red",
                                new DriveOverStationBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, false, false));
                mAutonomousChooser.addOption("Drive Over Station Balance Blue",
                                new DriveOverStationBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, true, false));
                mAutonomousChooser.addOption("Drive Over Station Balance Red Cone",
                                new DriveOverStationBalanceCone(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, false, false));
                mAutonomousChooser.addOption("Drive Over Station Balance Blue Cone",
                                new DriveOverStationBalanceCone(s_Swerve, mShoulder, mWrist, mBelt, mIntake,
                                                mCatapult, true, false));
                mAutonomousChooser.addOption("Drive Backward and Level (teamside)",
                                new DriveBackwardAndLevel(s_Swerve, true));

                mAutonomousChooser.addOption("Blue Top 2 Cycle",
                                new TopTwoCycle(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, true));
                mAutonomousChooser.addOption("Blue Top 2 Cycle Balance",
                                new TopTwoBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, true));
                mAutonomousChooser.addOption("Blue Top 2.5 Cycle Balance",
                                new TopTwoHalfBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, true));

                mAutonomousChooser.addOption("Blue Top 3 cycle Mid",
                                new TopThreeBack(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, true));

                mAutonomousChooser.addOption("Blue Top 3 cycle High",
                                new TopThreeBack(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, true,
                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE));

                
                mAutonomousChooser.addOption("Red Top 2 Cycle",
                                new TopTwoCycle(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, false));
                mAutonomousChooser.addOption("Red Top 2 Cycle Balance",
                                new TopTwoBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, false));
                mAutonomousChooser.addOption("Red Top 2.5 Cycle Balance",
                                new TopTwoHalfBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, false));
                mAutonomousChooser.addOption("Red Top 3 cycle High",
                                new TopThreeBack(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, false,
                                                Constants.ArmPoses.BACKWARDS_HIGH_CONE));

                mAutonomousChooser.setDefaultOption("Red Top 3 cycle Mid",
                                new TopThreeBack(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, false));
                mAutonomousChooser.addOption("Blue Bottom 2 Cycle",
                                new BottomTwoCycleTag(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, true));
                mAutonomousChooser.addOption("Red Bottom 2 Cycle",
                                new BottomTwoCycleTag(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult, false));
                mAutonomousChooser.addOption("Blue Bottom 2 Cycle and Balance",
                                new BottomTwoCycleTagBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult,
                                                true));
                mAutonomousChooser.addOption("Blue Bottom 2.5",
                                new BottomTwoHalfCycle(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult,
                                                true));
                mAutonomousChooser.addOption("Red Bottom 2.5",
                                new BottomTwoHalfCycle(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult,
                                                false));
                mAutonomousChooser.addOption("Red Bottom 2 Cycle and Balance",
                                new BottomTwoCycleTagBalance(s_Swerve, mShoulder, mWrist, mBelt, mIntake, mCatapult,
                                                false));
                Shuffleboard.getTab("Autonomous").add(mAutonomousChooser);

                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> -mDriverController.getRawAxis(translationAxis),
                                                () -> -mDriverController.getRawAxis(strafeAxis),
                                                () -> -mDriverController.getRawAxis(rotationAxis),
                                                () -> robotCentric.getAsBoolean(),
                                                () -> mDriverController.getLeftBumper(),
                                                () -> mDriverController
                                                                .getLeftTriggerAxis() > Constants.ControllerInputs.DEADBAND));

                mShoulder.setDefaultCommand(
                                new SetShoulderSpeed(mShoulder, () -> mOperatorController.getLeftBumper() ? 0.0
                                                : -mOperatorController.getLeftY() / 3.0));
                mWrist.setDefaultCommand(
                                new SetWristSpeed(mWrist, () -> -mOperatorController.getRightY() / 2.0));

                mBelt.setDefaultCommand(new SetBeltSpeed(mBelt,
                                () -> mOperatorController.getLeftBumper() ? -mOperatorController.getLeftY() / 3.0
                                                : 0.0));
                
                mLED.setDefaultCommand(new RunLED(mLED, () -> mDriverController.getRightBumper(),
                                () -> mDriverController.getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD,
                                () -> mIntake.hasGamePiece()));
                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /* PROGRAMMER CONTROLLER */
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //Comment and Uncomment Code as Necessary for Testing Robot
                //Some Driver Buttons are Commented per Driver Preference
                // RIGHT TRIGGER - Align to Tag Forward

                // Trigger mProgrammingRT = new Trigger(() ->
                // mProgrammingController.getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD);
                // mProgrammingRT.
                Trigger mProgrammingRB = new Trigger(() -> mProgrammingController.getRightBumper());
                // mProgrammingRB.onTrue(new DriveToPositionAbsolute(s_Swerve,()->new
                // Pose2d()));
                // mProgrammingRB.whileTrue(new DriveToPositionAbsolute(s_Swerve, getScoringLocation(s_Swerve, 0.0)));
                Trigger mProgrammingLB = new Trigger(() -> mProgrammingController.getLeftBumper());
                // mProgrammingRB.onTrue(new DriveToPositionAbsolute(s_Swerve,()->new
                // Pose2d()));
                // mProgrammingLB.whileTrue(
                //                 new DriveToPositionAbsolute(s_Swerve,
                //                                 getScoringLocation(s_Swerve, -Constants.Field.TAG_TO_SIDE)));

                // X BUTTON - Move wrist to setpoint
                Trigger mProgrammerXButton = new Trigger(() -> mProgrammingController.getXButton());
                // mProgrammerXButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                //                 Constants.ArmPoses.FORWARD_MID_CUBE));

                // B Button - launch stuff
                Trigger mProgrammerBButton = new Trigger(() -> mDriverController.getBButton());
                mProgrammerBButton.whileTrue(new InstantCommand(() -> mCatapult.launch(), mCatapult));

                Trigger mProgrammerAButton = new Trigger(() -> mDriverController.getAButton());
                mProgrammerAButton.whileTrue(new InstantCommand(() -> mCatapult.unLaunch(), mCatapult));

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /* DRIVER CONTROLLER */
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                // START BUTTON - Reset the drivebase gyro and encoders
                Trigger mDriverStartButton = new Trigger(() -> mDriverController.getStartButton());
                mDriverStartButton.whileTrue(new ResetAll(s_Swerve));

                // BACK BUTTON - lock the drivebase wheels into a diamond formation

                // A BUTTON - sets the swerve to hold it's heading to be facing our
                // driverstation
                // Trigger mDriverAButton = new Trigger(() -> mDriverController.getAButton());
                // mDriverAButton.onTrue(new SetSwerveHeading(s_Swerve, Rotation2d.fromDegrees(180.)));
                // mDriverAButton.onTrue(new TurnToAngle(s_Swerve, 180.0));

                // X BUTTON - sets the swerve to hold it's heading to be pointing to the right
                // Trigger mDriverXButton = new Trigger(() -> mDriverController.getXButton())
                // .and(() -> !(mDriverController.getRightTriggerAxis() >
                // Constants.TRIGGER_THRESHOLD));
                // mDriverXButton.onTrue(new SetSwerveHeading(s_Swerve,
                // Rotation2d.fromDegrees(90.)));
                // mDriverXButton.onTrue(new TurnToAngle(s_Swerve, 270.0));

                // Y BUTTON - sets the swerve to hold it's heading to be pointing down field
                Trigger mDriverYButton = new Trigger(() -> mDriverController.getYButton());
                mDriverYButton.onTrue(new SetSwerveHeading(s_Swerve, Rotation2d.fromDegrees(0.)));
                // mDriverYButton.onTrue(new TurnToAngle(s_Swerve, 0.0));

                // B BUTTON - sets the swerve to hold it's heading to be pointing to the left
                // Trigger mDriverBButton = new Trigger(() -> mDriverController.getBButton())
                // .and(() -> !(mDriverController.getRightTriggerAxis() >
                // Constants.TRIGGER_THRESHOLD));
                // mDriverBButton.whileTrue(new SetSwerveHeading(s_Swerve,
                // Rotation2d.fromDegrees(270.)));
                // mDriverBButton.onTrue(new TurnToAngle(s_Swerve, 90.0));

                // RIGHT TRIGGER + X BUTTON - Align to the left of the closest April Tag
                Trigger mDriverXandRT = new Trigger(
                                () -> mDriverController.getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD)
                                .and(() -> mDriverController.getXButton()).debounce(0.05, DebounceType.kRising);
                mDriverXandRT.whileTrue(new DriveToPositionAbsolute(s_Swerve,
                                getScoringLocation(s_Swerve, Constants.Field.TAG_TO_SIDE, Units.inchesToMeters(-6.))));

                // RIGHT TRIGGER + B BUTTON - Align to the right of the closest April Tag
                Trigger mDriverBandRT = new Trigger(
                                () -> mDriverController.getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD)
                                .and(() -> mDriverController.getBButton()).debounce(0.05, DebounceType.kRising);
                mDriverBandRT.whileTrue(
                                new DriveToPositionAbsolute(s_Swerve,
                                                getScoringLocation(s_Swerve,
                                                                -Constants.Field.TAG_TO_SIDE - Units.inchesToMeters(
                                                                                3.0),
                                                                Units.inchesToMeters(-6.))));

                // RIGHT TRIGGER - Align to the closest April Tag
                Trigger mDriverRT = new Trigger(
                                () -> mDriverController.getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD)
                                .and(() -> !mDriverController.getXButton() && !mDriverController.getBButton())
                                .debounce(0.05, DebounceType.kRising);
                mDriverRT.whileTrue(new DriveToPositionAbsolute(s_Swerve,
                                getScoringLocation(s_Swerve, 0.0, Units.inchesToMeters(-6.))));

                Trigger mDriverRB = new Trigger(() -> mDriverController.getRightBumper());
                mDriverRB.whileTrue(new DiamondFormation(s_Swerve));


                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /* OPERATOR CONTROLLER */
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                // BACK + RB + LB BUTTON - Stardust (full reset arm)
                Trigger mLightsaber = new Trigger(() -> mOperatorController.getBackButton())
                                .and(() -> mOperatorController.getRightBumper())
                                .and(() -> mOperatorController.getLeftBumper());
                mLightsaber.whileTrue(new InstantCommand(() -> mShoulder.ResetShoulderPosition(), mShoulder));


                // BACK BUTTON - Home the belt stage to zero
                Trigger mOperatorBack = new Trigger(() -> mOperatorController.getBackButton());
                mOperatorBack.onTrue(new HomeBeltToZero(mBelt));

                // START BUTTON - Stop the arm state machine from running
                Trigger mOperatorStart = new Trigger(() -> mOperatorController.getStartButton());
                mOperatorStart.whileTrue(new TurnStateMachineOff(mShoulder, mWrist, mBelt));

                // A BUTTON - INTAKE, moves the arm to floor load
                Trigger mOperatorAButton = new Trigger(() -> mOperatorController.getAButton()); // intake
                mOperatorAButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake, Pose.INTAKE));

                // Y BUTTON - BACKWARDS HIGH, moves the arm to score over the back on the high
                // row
                Trigger mOperatorYButton = new Trigger(() -> mOperatorController.getYButton()); // backward high
                mOperatorYButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Pose.BACKWARDS_HIGH));

                // X BUTTON - BACKWARDS MID, moves the arm to score over the back on the mid row
                Trigger mOperatorXButton = new Trigger(() -> mOperatorController.getXButton())
                                .and(() -> !mOperatorController.getLeftBumper()); // backward mid
                mOperatorXButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Pose.BACKWARDS_MID));

                Trigger mOperatorLeftStick = new Trigger(() -> mOperatorController.getLeftStickButton()); // backward
                                                                                                          // mid
                mOperatorLeftStick.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                ArmPoses.AUTON_HIGH));

                // B BUTTON - STOW, moves the arm into a position that is save to quickly drive
                // around the field with
                Trigger mOperatorBButton = new Trigger(() -> mOperatorController.getBButton())
                                .and(() -> !mOperatorController.getLeftBumper()); // stow
                mOperatorBButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Constants.ArmPoses.STOW));

                // LEFT BUMPER + X BUTTON - FORWARD MID, moves the arm to score forward on the
                // mid row
                Trigger mOperatorXandLBButton = new Trigger(() -> mOperatorController.getLeftBumper())
                                .and(() -> mOperatorController.getXButton()); // forward mid
                mOperatorXandLBButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Constants.ArmPoses.ABSOLUTE_STOW));

                Trigger mOperatorAandLBButton = new Trigger(() -> mOperatorController.getLeftBumper())
                                .and(() -> mOperatorController.getAButton()); // forward mid
                mOperatorAandLBButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Constants.ArmPoses.DROP));

                Trigger mOperatorYandLBButton = new Trigger(() -> mOperatorController.getLeftBumper())
                                .and(() -> mOperatorController.getYButton()); // forward mid
                mOperatorYandLBButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Constants.ArmPoses.SHELF));

                Trigger mOperatorBandLBButton = new Trigger(() -> mOperatorController.getLeftBumper())
                                .and(() -> mOperatorController.getBButton()); // forward mid
                mOperatorBandLBButton.onTrue(new ArmStateMachine(mShoulder, mWrist, mBelt, mIntake,
                                Constants.ArmPoses.LOW_SCORE));

                // RIGHT BUMPER - TOGGLE CONE/CUBE SCORING
                Trigger mOperatorRB = new Trigger(() -> mOperatorController.getRightBumper());
                mOperatorRB.onTrue(new InstantCommand(() -> Intake.coneMode = !Intake.coneMode));

                // RIGHT TRIGGER - Score game piece
                Trigger mOperatorRT = new Trigger(
                                () -> mOperatorController.getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD);
                mOperatorRT.whileTrue(new OuttakePiece(mIntake));

                // LEFT TRIGGER - Intake game piece
                Trigger mOperatorLT = new Trigger(
                                () -> (mOperatorController.getLeftTriggerAxis() > Constants.TRIGGER_THRESHOLD));
                mOperatorLT.whileTrue(
                                new RepeatCommand(new IntakePiece(mIntake, () -> mOperatorController.getLeftBumper())));

                Trigger mOperatorRS = new Trigger(
                                () -> (mOperatorController.getRightStickButton()));
                mOperatorRS.onTrue(new Fling(mShoulder, mWrist, mBelt, mIntake));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         * @throws FileNotFoundException
         */
        public Command getAutonomousCommand() {
                return mAutonomousChooser.getSelected();
        }

}
