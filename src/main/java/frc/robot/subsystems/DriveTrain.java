// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  public static DriveTrain driveTrain;

  // Setup our drive motors.
  static CANSparkMax leftMotorLeader = new CANSparkMax(Constants.Drive.FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  static CANSparkMax leftMotorFollower = new CANSparkMax(Constants.Drive.BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);

  static CANSparkMax rightMotorLeader = new CANSparkMax(Constants.Drive.FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  static CANSparkMax rightMotorFollower = new CANSparkMax(Constants.Drive.BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);

  /* From what I've read I have to give each wheel its own PIDController and control the four by themselves for minimal error. 
   * I just feel that this is odd but due to the abbe error and such others I have to take this approach.
   * I'm going to try it but keep a backup on-hand. -R
  */

  // PID CONTROLLER FOR DRIVER //
   //TODO: TUNE THESE
   private static final double kP = 0.1;
   private static final double kI = 0.0;
   private static final double kD = 0.0;
   private static final double kF = 0.0;

   private static final double kS = 0.0;  // Static feedforward gain
   private static final double kV = 0.0;  // Velocity feedforward gain
   private static final double kA = 0.0;  // Acceleration feedforward gain

   private SparkPIDController ml_pidController = leftMotorLeader.getPIDController();
   private SparkPIDController mr_pidController = rightMotorLeader.getPIDController();

   // Making the tabs here so I can check their values later.
   private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
   private GenericEntry p_gain, i_gain, d_gain, f_gain, skS, skV, skA;

  // Our actual drive controller.
  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  
  public DriveTrain() {

    // Setup the followers of the motors.
    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorLeader);

    //leftLeader.setSmartCurrentLimit();
    //rightLeader.setSmartCurrentLimit();

    // Mirrored motors, mirrored setup.
    rightMotorLeader.setInverted(false);
    leftMotorLeader.setInverted(true);

    //leftLeader.setOpenLoopRampRate(.85);
    //rightLeader.setOpenLoopRampRate(.85);

    leftMotorLeader.setIdleMode(IdleMode.kBrake);
    rightMotorLeader.setIdleMode(IdleMode.kBrake);
    leftMotorFollower.setIdleMode(IdleMode.kBrake);
    rightMotorFollower.setIdleMode(IdleMode.kBrake);

    if (Constants.Debug.debugMode) {
      // Add PID and feedforward constants to Shuffleboard.
      p_gain = tab.add("P Gain", ml_pidController.getP()).withWidget(BuiltInWidgets.kTextView).getEntry();
      i_gain = tab.add("I Gain", ml_pidController.getI()).withWidget(BuiltInWidgets.kTextView).getEntry();
      d_gain = tab.add("D Gain", ml_pidController.getD()).withWidget(BuiltInWidgets.kTextView).getEntry();
      f_gain = tab.add("FF Gain", ml_pidController.getFF()).withWidget(BuiltInWidgets.kTextView).getEntry();
      skS = tab.add("kS", kS).withWidget(BuiltInWidgets.kTextView).getEntry();
      skV = tab.add("kV", kV).withWidget(BuiltInWidgets.kTextView).getEntry();
      skA = tab.add("kA", kA).withWidget(BuiltInWidgets.kTextView).getEntry();
    }
  }

  // Easy way to get instances. Returns the drivetrain obj if it's not null.
  public static DriveTrain getInstance() {
    if (driveTrain == null) {driveTrain = new DriveTrain();}
    return driveTrain;
  }

  //The drive mode, don't ask why it's swapped.
  public void driveArcade (double forwardBackSpeed, double rotationSpeed) {
    differentialDrive.arcadeDrive(forwardBackSpeed, rotationSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Only updates in debug mode to prevent errors in production.
    if (Constants.Debug.debugMode) {
      // Update PID constants.
      //? VALUES NOT GRABBING????
      ml_pidController.setP(p_gain.getDouble(0.1));
      ml_pidController.setI(i_gain.getDouble(0));
      ml_pidController.setD(d_gain.getDouble(0)); 
      // Update feedforward constants.
      ml_pidController.setFF(f_gain.getDouble(0));
      //System.out.println(p_gain.getDouble(0.1) + i_gain.getDouble(0.1) + d_gain.getDouble(0.1) + f_gain.getDouble(0.1));
      //skS
      //skV
      //skA
    }
  }
}
