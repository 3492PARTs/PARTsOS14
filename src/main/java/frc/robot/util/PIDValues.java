// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** PIDValues
 * Double array with three spots in order:
 * @param kP - Proportional gain
 * @param kI - Integral gain
 * @param kD - Derivative gain
 * @return double[] - Array of PID values.
 */
public class PIDValues {
    double[] PIDValues = new double[3];

    public PIDValues(double kP, double kI, double kD) {
        this.PIDValues[0] = kP;
        this.PIDValues[1] = kI;
        this.PIDValues[2] = kD;
    }

    public double[] getPIDValues() {
        return PIDValues;
    }
}
