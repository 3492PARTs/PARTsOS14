
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

  /** Drive Constants */
  public static final class Drive {
    public static final int FRONT_RIGHT_MOTOR = 6;
    public static final int BACK_RIGHT_MOTOR = 5;

    public static final int FRONT_LEFT_MOTOR = 3;
    public static final int BACK_LEFT_MOTOR = 2;

    public static final double OPEN_LOOP_RATE = 0.8;
  }

  /** Arm Constants */
  public static final class Arm {
    public static final int LEFT_PIVOT_MOTOR = 4;
    public static final int RIGHT_PIVOT_MOTOR = 7;

    public static final double PIVOT_GEAR_RATIO = 234;
    public static final double OPEN_LOOP_RATE = 0.34;

    public static final double kP = 30.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double GROUND = 0;
    public static final double SPEAKER = 45.3;
    public static final double HOME = 75;
    public static final double AMP = 100;
    public static final double UPPER_BOUND = AMP + 10;
    //public static final double LOWER_BOUND = GROUND + 10;

    public static final double SPEAKER_SIDE_ANGLE = 41.0;

    public static final int L_SWITCH_PORT = 0;

    public static final boolean SYSID = false;
  }

  /** Intake Constants */
  public static final class Intake {
    public static final int INTAKE_MOTOR = 11;
    public static final double INTAKE_SPEED = -.8;
    public static final int PHOTOEYE = 9;
  }

  /** Intake Constants */
  public static final class Shooter {
    public static final int LEFT_MOTOR = 10;
    public static final int RIGHT_MOTOR = 12;

  }

  public static final class LED {
    //TODO: Replace with real stuff later.
    public static final int LED_PORT = 123;
    public static final int LED_LENGTH = 100;
  }

  // THIS IS FOR SHUFFLEBOARD DEBUG PLUS OTHER MISC THINGS //
  /** Debug Constants<p>For testingg only.<p>Do not use in prod! */
  public static class Debug {
    public static final boolean debugMode = true;
  }

}
