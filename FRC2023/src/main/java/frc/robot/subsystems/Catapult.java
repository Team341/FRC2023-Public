// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Catapult extends SubsystemBase {
  private static Catapult instance = null;
  
  
  private Solenoid mLaunchPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.CATAPULT.LAUNCHER_PORT);
  /** Creates a new Catapult. */
  public Catapult() {


  }
    /** Get an instance of Intake. */
    public static Catapult getInstance() {
      if (instance == null) {
        instance = new Catapult();
      }
      return instance;
    }

  public void launch() {
    mLaunchPiston.set(true);
    // mLaunchPiston2.set(true);
    
  }
  public void unLaunch() {
    mLaunchPiston.set(false);
    //mLaunchPiston2.set(false);

  }
  
  @Override
  public void periodic() {
    if (Constants.FMSDETACHED)
    SmartDashboard.putBoolean("Catapult/are we doing the launch?", mLaunchPiston.get()/*&&mLaunchPiston2.get() */);
    // This method will be called once per scheduler run
  }
}
