
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;

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

    public static final double turning_kP = 0.091;
    public static final double turning_kI = 0.0;//0.013;
    public static final double turning_kD = 0.0;//0.01;
  }

  /** Arm Constants */
  public static final class Arm {
    public static final int LEFT_PIVOT_MOTOR = 4;
    public static final int RIGHT_PIVOT_MOTOR = 8;

    public static final double PIVOT_GEAR_RATIO = 234;
    public static final double OPEN_LOOP_RATE = 0.34;

    // PID
    public static final double kP = 22.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    //Feedforward Radians
    public static final double kS = .43078; //0.21092; // .38107
    public static final double kV = 0.10925;//0.11019; // .10239
    public static final double kG = 0.4; //0.02501; // .02501
    public static final double kA = 0.016668;//0.021335;

    public static final double GROUND = 0;
    public static final double SPEAKER = 25.5;//28.0;
    public static final double HOME = 65.2;
    public static final double AMP = 83;
    public static final double UPPER_BOUND = AMP + 10;
    public static final double SPEAKER_BACK_30 = 37;
    public static final double AMP_NOTE_SPEAKER = 40;
    //public static final double LOWER_BOUND = GROUND + 10;

    //public static final double SPEAKER_SIDE_ANGLE = 41.0;

    //public static final int L_SWITCH_PORT = 0;

    public static final boolean SYSID = false;
  }

  /** Intake Constants */
  public static final class Intake {
    public static final int INTAKE_MOTOR = 11;
    public static final double INTAKE_SPEED = -.8;
    public static final int PHOTOEYE = 0;
  }

  /** Intake Constants */
  public static final class Shooter {
    public static final int LEFT_MOTOR = 10;
    public static final int RIGHT_MOTOR = 12;
    public static final double TOLERANCE = 0;
    public static final double LIMITER = 0.25;
    //TODO: check if rpm works with current peak limit
    public static final int SPEAKER_RPM = 800; //1550;
    public static final int WARMUP_SPEAKER_RPM = 700;
    public static final int AMP_RPM = 300;
  }

  public static final class Climber {
    public static final int LEFT_MOTOR = 18;
    public static final int RIGHT_MOTOR = 23;
  }

  public static final class LED {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 100;
  }

  public static final class Dashboard {
    public enum Tabs {

      PRE_MATCH("Pre Match"),
      AUTONOMOUS("Autonomous"),
      TELEOPERATED("Teleoperated"),
      DEBUG("Debug");

      public final String tabName;

      Tabs(String tabName) {
        this.tabName = tabName;
      }
    }
  }

  /** Debug Constants For testing only. Do not use in prod! */
  public static class Debug {
    public static final boolean debugMode = false;
    public static final boolean logging = false;
  }

  /**
   * Swerve Constants, in its own class to reserve original configuration.
   */
  public static final class Swerve {
    public static final int pigeonID = 0; // TODO: Replace with port for piegonID.
    
    //TODO: This must be tuned to specific robot
    public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

      /* Drivetrain Constants */
      public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
      public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
      public static final double wheelCircumference = chosenModule.wheelCircumference;

      /* Swerve Kinematics 
       * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
      public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

      /* Module Gear Ratios */
      public static final double driveGearRatio = chosenModule.driveGearRatio;
      public static final double angleGearRatio = chosenModule.angleGearRatio;

      /* Motor Inverts */
      public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
      public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

      /* Angle Encoder Invert */
      public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

      /* Swerve Current Limiting */
      public static final int angleCurrentLimit = 25;
      public static final int angleCurrentThreshold = 40;
      public static final double angleCurrentThresholdTime = 0.1;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveCurrentLimit = 35;
      public static final int driveCurrentThreshold = 60;
      public static final double driveCurrentThresholdTime = 0.1;
      public static final boolean driveEnableCurrentLimit = true;

      /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
       * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      /* Angle Motor PID Values */
      public static final double angleKP = chosenModule.angleKP;
      public static final double angleKI = chosenModule.angleKI;
      public static final double angleKD = chosenModule.angleKD;

      /* Drive Motor PID Values */
      public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;
      public static final double driveKF = 0.0;

      /* Drive Motor Characterization Values From SYSID */
      public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
      public static final double driveKV = 1.51;
      public static final double driveKA = 0.27;

      /* Swerve Profiling Values */
      /** Meters per Second */
      public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
      /** Radians per Second */
      public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

      /* Neutral Modes */
      public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
  }
}
