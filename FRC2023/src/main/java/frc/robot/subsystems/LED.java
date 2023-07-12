// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class LED extends SubsystemBase {

  private static LED mInstance;

  public static LED getInstance() {
    if (mInstance == null) {
      mInstance = new LED();
    }
    return mInstance;
  }

  private final CANdle mLED;
  private boolean isAuto;

  private CANdleConfiguration config;
  public boolean isAnimating = false;


  /** Creates a new LED. */
  public LED() {
    isAuto = false;
    mLED = new CANdle(Constants.LED.LED_PORT, Constants.CANIVORE_NAME);
    config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    config.vBatOutputMode = VBatOutputMode.On;

    mLED.configAllSettings(config, 100);
    // mLED.configFactoryDefault();
  }

  public void setAutoLED() {
    isAuto = true;
  }

  public void setTeleOpLED() {
    isAuto = false;
  }

  public boolean isUsingAutoLED() {
    return isAuto;
  }

  public void setAllLEDs(Color color) {
    setRightLEDs(color);
    setLeftLEDs(color);
  }

  public void setCANDLELED(Color color) {
    mLED.setLEDs((int) color.red, (int) color.green, (int) color.blue, 255, Constants.LED.CANDLE_START_INDEX,
        Constants.LED.CANDLE_LED_LENGTH);
  }

  public void setRightLEDs(Color color) {
    mLED.clearAnimation(0);
    mLED.setLEDs((int) color.red, (int) color.green, (int) color.blue, 255, Constants.LED.RIGHT_START_INDEX,
        Constants.LED.RIGHT_LED_LENGTH);
  }

  public void setLeftLEDs(Color color) {
    mLED.clearAnimation(0);
    mLED.setLEDs((int) color.red, (int) color.green, (int) color.blue, 255, Constants.LED.LEFT_START_INDEX,
        Constants.LED.LEFT_LED_LENGTH);
  }

  // Wrapper functions

  /**
   * @return bus voltage
   */
  public double getBatteryVoltage() {
    return mLED.getBusVoltage();
  }

  /**
   * @return rail voltage
   */
  public double get5V() {
    return mLED.get5VRailVoltage();
  }

  /**
   * @return low side current
   */
  public double getCurrent() {
    return mLED.getCurrent();
  }

  /**
   * @return temperature in celcius
   */
  public double getTemperature() {
    return mLED.getTemperature();
  }

  /**
   * @param percent scaling brightness
   */
  public void configBrightness(double percent) {
    mLED.configBrightnessScalar(percent, 0);
  }

  /**
   * @param disableWhenLos whether to disable LED when signal is lost
   */
  public void configLos(boolean disableWhenLos) {
    mLED.configLOSBehavior(disableWhenLos, 0);
  }

  /**
   * @param type the type of LED
   */
  public void configLedType(LEDStripType type) {
    mLED.configLEDType(type, 0);
  }

  /**
   * @param offWhenActive whether LED is off when CANdle is activated
   */
  public void configStatusLedBehavior(boolean offWhenActive) {
    mLED.configStatusLedState(offWhenActive, 0);
  }

  public static double last_time = Timer.getFPGATimestamp();
  public static int cor = 0;
 

  public boolean goingDown = false;
  public int cycleIndex = 0;
  public int[] backLEDDisplay = {3,4,1};
  public double actualLastTime = Timer.getFPGATimestamp();

  
  public void updateAnimation() {
    if (isAnimating) {
      getLED().animate(Constants.LED.WIN_BM, 0);
      return;
    }
    double cur = Timer.getFPGATimestamp() - last_time;
    if (cur >= 1.0) {
      last_time = cur;
      if (Timer.getFPGATimestamp() - actualLastTime >= 0.5) {
        actualLastTime = Timer.getFPGATimestamp();
        mLED.setLEDs(0, 0, 0, 0, 0, 8);
        mLED.setLEDs(0, 0, 255, 0, 4, backLEDDisplay[cycleIndex%3]);
        cycleIndex++;
      }
      
      
    //Different LED Animation
    // mLED.setLEDs((int)(Math.random()*255), (int)(Math.random()*255), (int)(Math.random()*255),(int)(Math.random()*255),0, (int)(Math.random()*9));

      // if (goingDown) {
      //   cor -= 1;
      //   mLED.setLEDs(255, 255, 0, 0, Constants.LED.LEFT_START_INDEX, cor);
      //   // mLED.setLEDs(255, 255, 0, 0, Constants.LED.RIGHT_START_INDEX, cor);
      //   mLED.setLEDs(255, 255, 0, 0, Constants.LED.RIGHT_START_INDEX+Constants.LED.RIGHT_LED_LENGTH-cor, cor);

      //   //mLED.setLEDs(0, 0, 255, 0, Constants.LED.RIGHT_START_INDEX + cor, 1);
      //   mLED.setLEDs(0, 0, 255, 255, Constants.LED.LEFT_START_INDEX + cor, 1);
      //   mLED.setLEDs(0, 0, 255, 0, Constants.LED.RIGHT_START_INDEX + Constants.LED.RIGHT_LED_LENGTH-cor, 1);

      //   if (cor == 0) {
      //     goingDown = false;
      //   }
      // } else {

      //   mLED.setLEDs(255, 255, 0, 0, Constants.LED.LEFT_START_INDEX, cor);
      //   //mLED.setLEDs(255, 255, 0, 0, Constants.LED.RIGHT_START_INDEX, cor);
      //   mLED.setLEDs(255, 255, 0, 0, Constants.LED.RIGHT_START_INDEX+Constants.LED.RIGHT_LED_LENGTH-cor, cor);


      //   cor += 1;

      //   if (cor == Constants.LED.LEFT_LED_LENGTH) {
      //     goingDown = true;
      //   }
      // }
        cor += 1;
      

        for(int j=0; j<=25; j++){
        if((cor+j)%12<6){
          if((int)(Math.random()*50000)==42069){
            mLED.setLEDs(188, 14, 188, 0, Constants.LED.LEFT_START_INDEX+j, 1);
            mLED.setLEDs(188, 14, 188, 0, Constants.LED.RIGHT_LED_LENGTH+ Constants.LED.RIGHT_START_INDEX-j-1, 1);
           }
           
          mLED.setLEDs(255, 255, 0, 0, Constants.LED.LEFT_START_INDEX+j, 1);
          mLED.setLEDs(255, 255, 0, 0, Constants.LED.RIGHT_LED_LENGTH+ Constants.LED.RIGHT_START_INDEX-j-1, 1);
        
        } else {

          if((int)(Math.random()*50000)==42069){
            mLED.setLEDs(188, 14, 188, 0, Constants.LED.LEFT_START_INDEX+j, 1);
            mLED.setLEDs(188, 14, 188, 0, Constants.LED.RIGHT_LED_LENGTH+ Constants.LED.RIGHT_START_INDEX-j-1, 1);
          }
          mLED.setLEDs(0,0, 255, 0, Constants.LED.LEFT_START_INDEX+j, 1);
          mLED.setLEDs(0,0, 255, 0, Constants.LED.RIGHT_LED_LENGTH+ Constants.LED.RIGHT_START_INDEX-j-1, 1);
         }
        }
           
    }
  }

  private void logToDashboard() {

  }

  private void logToDashboardInDetail() {
     SmartDashboard.putNumber("LED/Current", getCurrent());
     SmartDashboard.putNumber("LED/Temperature", getTemperature());
     SmartDashboard.putNumber("LED/Battery Voltage", getBatteryVoltage());
     SmartDashboard.putNumber("LED/5V Rail Voltage", get5V());
   
  }

  /**
   * @return LED
   */
  public CANdle getLED() {
    return mLED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.detailedLogging) {
      logToDashboardInDetail();
    }
    if (Robot.logging) {
      logToDashboard();
    }

  }

  public void clear() {
    getLED().clearAnimation(0);
    setAllLEDs(Color.kWhite);
  }

  public void clearAnimation() {
    getLED().clearAnimation(0);

    setLeftLEDs(Color.kWhite);
    setRightLEDs(Color.kWhite);

  }
}