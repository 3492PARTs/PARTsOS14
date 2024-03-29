
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

    public static final double OPEN_LOOP_RATE = 0.7;
    public static final double kP = 8.2;
    public static final double kI = 0.1;
    public static final double kD = 0.6;

    public static final double turning_kP = .073;//0.07;
    public static final double turning_kI = 0.0;//0.013;
    public static final double turning_kD = 0.0;//0.01;
  }

  /** Arm Constants */
  public static final class Arm {
    public static final int LEFT_PIVOT_MOTOR = 4;
    public static final int RIGHT_PIVOT_MOTOR = 7;

    public static final double PIVOT_GEAR_RATIO = 234;
    public static final double OPEN_LOOP_RATE = 0.34;

    // PID
    public static final double kP = 30.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    //Feedforward
    public static final double kS = 0.02204; //0.21092; // .38107
    public static final double kV = 0.15682;//0.11019; // .10239
    public static final double kG = 0.65433; //0.02501; // .02501
    public static final double kA = 0.027191;//0.021335;

    public static final double GROUND = 0;
    public static final double SPEAKER = 28.0;
    public static final double HOME = 65.2;
    public static final double AMP = 100;
    public static final double UPPER_BOUND = AMP + 10;
    public static final double SPEAKER_BACK_30 = 37;
    //public static final double LOWER_BOUND = GROUND + 10;

    //public static final double SPEAKER_SIDE_ANGLE = 41.0;

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
    public static final double TOLERANCE = 0;
    public static final double LIMITER = 0.25;
    public static final int SPEAKER_RPM = 800; //1550;
    public static final int WARMUP_SPEAKER_RPM = 700;
    public static final int AMP_RPM = 300;
  }

  public static final class Climber {
    public static final int LEFT_MOTOR = 8;
    public static final int RIGHT_MOTOR = 9;
  }

  public static final class LED {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 100;
  }

  // THIS IS FOR SHUFFLEBOARD DEBUG PLUS OTHER MISC THINGS //
  /** Debug Constants<p>For testingg only.<p>Do not use in prod! */
  public static class Debug {
    public static final boolean debugMode = true;
  }

}
