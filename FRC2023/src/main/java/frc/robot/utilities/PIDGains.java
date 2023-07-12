// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class PIDGains {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    public double kV;
    public double kS;

    public double kMinOutput;
    public double kMaxOutput;

    public String mName;

    public PIDGains(double p, double i, double d, double f, double v, double s) {
        this(p, i, d, f, v, s, 0.0, 0.0);
    }

    public PIDGains(double p, double i, double d, double f, double v, double s, double min, double max) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        kV = v;
        kS = s;
        kMinOutput = min;
        kMaxOutput = max;
    }

    public PIDGains(double p, double i, double d, double f, double v, double s, double min, double max, String name) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        kV = v;
        kS = s;
        kMinOutput = min;
        kMaxOutput = max;
        mName = name;
    }

    public PIDGains(double p, double i, double d, double f) {
        this(p, i, d, f, 0.0, 0.0);
    }

    public PIDGains(double p, double i, double d) {
        this(p, i, d, 0.0, 0.0, 0.0);
    }

    public PIDGains(double p) {
        this(p, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void logToDashboard() {
        if (mName == null) {
            return;
        }
        // SmartDashboard.putNumber(mName + "/kP", kP);
        // SmartDashboard.putNumber(mName + "/kI", kI);
        // SmartDashboard.putNumber(mName + "/kD", kD);
        // SmartDashboard.putNumber(mName + "/kF", kF);
        // SmartDashboard.putNumber(mName + "/kS", kS);
        // SmartDashboard.putNumber(mName + "/kV", kV);
    }

    public void updateFromDashboard() {
        if (mName == null) {
            return;
        }
    if (Constants.FMSDETACHED)
        {
        kP = SmartDashboard.getNumber(mName + "/kP", kP);
        kI = SmartDashboard.getNumber(mName + "/kI", kI);
        kD = SmartDashboard.getNumber(mName + "/kD", kD);
        kF = SmartDashboard.getNumber(mName + "/kF", kF);
        kS = SmartDashboard.getNumber(mName + "/kS", kS);
        kV = SmartDashboard.getNumber(mName + "/kV", kV);
        }
    }
}
