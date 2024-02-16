// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // TODO: REPLACE PLACEHOLDERS

  public static final class Drive {
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 15;
    public static final int BACK_RIGHT_DRIVE_MOTOR = 24;

    public static final int FRONT_LEFT_DRIVE_MOTOR = 21;
    public static final int BACK_LEFT_DRIVE_MOTOR = 12;

    public static final double OPEN_LOOP_RATE = 0.8;

    //last year was 44/30
    //public static final double DRIVE_GEAR_RATIO = null;
  }
  

  public static final class Arm {
    public static final int LEFT_PIVOT_MOTOR = 7;
    public static final int RIGHT_PIVOT_MOTOR = 4;

    public static final double PIVOT_GEAR_RATIO = 44/12;
    public static final double OPEN_LOOP_RATE = 0.8;
  }


  public static final class Intake {
      public static final int INTAKE_MOTOR = 27;
      public static final double INTAKE_SPEED = .5;
  }


  public static final class Shooter {
    // These should be the values but are most likley in the wrong order.
    public static final int LEFT_SHOOTER_MOTOR = 5;
    public static final int RIGHT_SHOOTER_MOTOR = 1;

  }


  public static class Operator {
    //? Unkown for now.
    public static final int kDriverControllerPort = 0;
  }

  // THIS IS FOR SHUFFLEBOARD DEBUG PLUS OTHER MISC THINGS //
  public static class Debug {
    public static final boolean debugMode = true;
  }


}
