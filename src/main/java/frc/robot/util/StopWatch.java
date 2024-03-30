// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class StopWatch {
    private long startTime = 0;
    private long stopTime = 0;

    public void start() {
        startTime = System.currentTimeMillis();
        stopTime = 0;
    }

    public void stop() {
        stopTime = System.currentTimeMillis();
    }

    public void reset() {
        startTime = 0;
        stopTime = 0;
    }

    /**
     * Get the time in milliseconds since start, either as of the current time or as of the stop time if set
     * @return time in milliseconds
     */
    public long getMilliseconds() {
        long time = stopTime > 0 ? time = stopTime : System.currentTimeMillis();
        return time - startTime;
    }

    /**
     * Get the time in seconds since start, either as of the current time or as of the stop time if set
     * @return time in seconds
     */
    public double getSeconds() {
        return getMilliseconds() / 1000.0;
    }

    /**
     * Get the time in minutes since start, either as of the current time or as of the stop time if set
     * @return time in minutes
     */
    public double getMinutes() {
        return getSeconds() / 60;
    }

}
