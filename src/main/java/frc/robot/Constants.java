// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Drive {
    public static final int FRONT_RIGHT_MOTOR = 6;
    public static final int BACK_RIGHT_MOTOR = 5;

    public static final int FRONT_LEFT_MOTOR = 3;
    public static final int BACK_LEFT_MOTOR = 2;

    public static final double OPEN_LOOP_RATE = 0.8;
  }

  public static final class Arm {
    public static final int LEFT_PIVOT_MOTOR = 4;
    public static final int RIGHT_PIVOT_MOTOR = 7;

    public static final double PIVOT_GEAR_RATIO = 234;
    public static final double OPEN_LOOP_RATE = 0.8;

    public static final double kP = 2.7;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double GROUND = 5;
    public static final double SPEAKER = 42.8;
    public static final double HOME = 10;
    public static final double AMP = -5.09;
  }

  public static final class Intake {
    public static final int INTAKE_MOTOR = 11;
    public static final double INTAKE_SPEED = .8;
    public static final int PHOTOEYE = 0;
  }

  public static final class Shooter {
    public static final int LEFT_MOTOR = 10;
    public static final int RIGHT_MOTOR = 12;

  }

  // THIS IS FOR SHUFFLEBOARD DEBUG PLUS OTHER MISC THINGS //
  public static class Debug {
    public static final boolean debugMode = true;
  }

}
