package frc.robot.utilities;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Creates an XboxController with predefined methods for finding the state/value of Xbox controller
 * buttons/axes
 *
 * @author James Decker
 */
public class XboxController extends GenericHID {

  // Xbox Controller Button mapping
  public final static int A_Button = 1, B_Button = 2, X_Button = 3, Y_Button = 4, Left_Bumper = 5,
      Right_Bumper = 6, Back_Button = 7, Start_Button = 8, Left_Stick = 9, Right_Stick = 10,

  // Xbox Axes mapping::
  Axis_LeftX = 0, Axis_LeftY = 1, Axis_LeftTrigger = 2, Axis_RightTrigger = 3, Axis_RightX = 4,
      Axis_RightY = 5,
  // Not reccomended: buggy and unreliable
  Axis_DPad = 6;
  
  private Joystick controller;
  public JoystickButton ButtonA, ButtonB, ButtonX, ButtonY, BumperLeft, BumperRight, ButtonStart, ButtonBack; 

  public Timer rumbleTimer;
  
  /**
   * Constructs a new XboxController with a specified port
   */
  public XboxController(final int port) {
    super(port);
    
    controller = new Joystick(port);
    ButtonA = new JoystickButton(controller, Button.kA.value);
    ButtonB = new JoystickButton(controller, Button.kB.value);
    ButtonX = new JoystickButton(controller, Button.kX.value);
    ButtonY = new JoystickButton(controller, Button.kY.value);
    ButtonStart = new JoystickButton(controller, Button.kStart.value);
    ButtonBack = new JoystickButton(controller, Button.kBack.value);
    // BumperRight = new JoystickButton(controller, BumperRight.get());
    // BumperLeft = new JoystickButton(controller, BumperLeft.get());
  }

  public boolean getStartButton() {
    return getRawButton(Start_Button);
  }

  public boolean getBackButton() {
    return getRawButton(Back_Button);
  }

  public Joystick getController() {
    return controller;
  }

  /**
   * returns the current state of the A Button
   */
  public boolean getAButton() {
    return getRawButton(A_Button);
  }

  /**
   * returns the current state of the B Button
   */
  public boolean getBButton() {
    return getRawButton(B_Button);
  }

  /**
   * returns the current state of the X_Button
   */
  public boolean getXButton() {
    return getRawButton(X_Button);
  }

  /**
   * returns the current state of the Y_Button
   */
  public boolean getYButton() {
    return getRawButton(Y_Button);
  }

  /**
   * returns the current state of the Left_Bumper
   */
  public boolean getLB() {
    return getRawButton(Left_Bumper);
  }

  /**
   * returns the current state of the Right_Bumper
   */
  public boolean getRB() {
    return getRawButton(Right_Bumper);
  }

  public boolean getLeftTrigger() {
    return getRawAxis(Axis_LeftTrigger) != 0.0;
  }

  public boolean getRightTrigger() {
    return getRawAxis(Axis_RightTrigger) >= 0.1;
  }

  /**
   * returns the current state of the Left_Stick
   */
  public boolean getLeftStickClick() {
    return getRawButton(Left_Stick);
  }

  /**
   * returns the current state of the Right_Stick
   */
  public boolean getRightStickClick() {
    return getRawButton(Right_Stick);
  }

  /**
   * returns the current value of the Axis_LeftX
   */
  public double getLeftXAxis() {
    return getRawAxis(Axis_LeftX);
  }

  /**
   * returns the current value of the Axis_LeftY
   */
  public double getLeftYAxis() {
    return getRawAxis(Axis_LeftY);
  }

  /**
   * returns the current value of the Axis_RightX
   */
  public double getRightXAxis() {
    return getRawAxis(Axis_RightX);
  }

  /**
   * returns the current value of the Axis_RightY
   */
  public double getRightYAxis() {
    return getRawAxis(Axis_RightY);
  }
  
  /**
   * returns the current value of the Axis_LeftX
   */
  public double getDeadbandedLeftXAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_LeftX), deadband);
  }

  /**
   * returns the current value of the Axis_LeftY
   */
  public double getDeadbandedLeftYAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_LeftY), deadband);
  }

  /**
   * returns the current value of the Axis_RightX
   */
  public double getDeadbandedRightXAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_RightX), deadband);
  }

  /**
   * returns the current value of the Axis_RightY
   */
  public double getDeadbandedRightYAxis(double deadband) {
    return applyDeadband(getRawAxis(Axis_RightY), deadband);
  }

  public double getCubedLeftYAxis(double deadband) {
    return Math.pow(getDeadbandedLeftYAxis(deadband), 3);
  }

  public double getCubedLeftXAxis(double deadband) {
    return Math.pow(getDeadbandedLeftXAxis(deadband), 3);
  }

  public double getCubedRightYAxis(double deadband) {
    return Math.pow(getDeadbandedRightYAxis(deadband), 3);
  }

  public double getCubedRightXAxis(double deadband) {
    return Math.pow(getDeadbandedRightXAxis(deadband), 3);
  }

  public double getCubedRightTrigger() {
    return Math.pow(getRawAxis(Axis_RightTrigger), 3);
  }

  public double getCubedLeftTrigger() {
    return Math.pow(getRawAxis(Axis_LeftTrigger), 3);
  }

  /**
   * returns the current value of the Axis_DPad (NOTE:: Very buggy, not recommended)
   */
  public double getDPadAxis() {
    return getRawAxis(Axis_DPad);
  }
  
  public static double applyDeadband(double value, double deadband) {
    if (value > -deadband && value < deadband) {
      return 0.0;
    } else {
      return value;
    }
  }
  
  /**
   * Makes the controller rumble.
   * @param l The left rumble value.
   * @param r The right rumble value.
   */
  public void rumble(double l, double r) {
      setRumble(RumbleType.kLeftRumble, l);
      setRumble(RumbleType.kRightRumble, r);
  }
  
  /**
   * Makes the controller rumble for X seconds.
   * @param d The left rumble value.
   * @param e The right rumble value.
   * @param seconds How long the controller should rumble.
   */
  
  public void rumble(double d, double e, double seconds) {
      rumble(d, e);
      rumbleTimer = new Timer(seconds, false, new TimerUser() {
          public void timer() {
              rumble(0, 0);
          }
          public void timerStop() {
              rumbleTimer = null;
          }
      }).start();
  }

  // @Override
  // public double getX(Hand ) {
  //   return 0;
  // }

  // @Override
  // public double getY() {
  //   return 0;
  // }  
}
