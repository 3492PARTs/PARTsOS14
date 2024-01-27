// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // TODO: REPLACE NULL WITH REAL VALUES!

  public static final class Drive {

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 0;
    public static final int BACK_RIGHT_DRIVE_MOTOR = 1;

    public static final int FRONT_LEFT_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_DRIVE_MOTOR = 4;

  }
  

  public static final class Elevator {
    public static final int LEFT_PIVOT_MOTOR = 5;
    public static final int RIGHT_PIVOT_MOTOR = 6;
    public static final double PIVOT_GEAR_RATIO = 7;
  }


  public static final class Intake {
      public static final int INTAKE_MOTOR = 29;
  }


  public static final class Shooter {
    public static final int LEFT_SHOOTER_MOTOR = 9;
    public static final int RIGHT_SHOOTER_MOTOR = 10;
  }


  public static class OperatorConstants {
    
    public static final int kDriverControllerPort = 0;

  }


}
