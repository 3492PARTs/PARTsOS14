// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public class Logger {
    private static Logger loggerInstance;
    private static DataLog log;

    public Logger() {
        if (Constants.Debug.logging) {
            // Starts recording to data log
            DataLogManager.start();

            log = DataLogManager.getLog();
            // Record both DS control and joystick data
            //DriverStation.startDataLog(log);
        }
    }

    public static Logger getInstance() {
        // If instance is null, then make a new instance.
        if (loggerInstance == null) {
            loggerInstance = new Logger();
        }
        return loggerInstance;
    }

    public boolean logBoolean(String key, boolean b) {
        if (Constants.Debug.logging) {
            new BooleanLogEntry(log, key).append(b);
            return true;
        } else
            return false;
    }

    public boolean logDouble(String key, double d) {
        if (Constants.Debug.logging) {
            new DoubleLogEntry(log, key).append(d);
            return true;
        } else
            return false;
    }

    public boolean logString(String key, String s) {
        if (Constants.Debug.logging) {
            new StringLogEntry(log, key).append(s);
            return true;
        } else
            return false;
    }

}
