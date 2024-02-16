// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  public static DriveTrain driveTrain;

  /* Setup the motors. */
  static CANSparkMax leftMotorLeader = new CANSparkMax(Constants.Drive.FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
  static CANSparkMax leftMotorFollower = new CANSparkMax(Constants.Drive.BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);

  static CANSparkMax rightMotorLeader = new CANSparkMax(Constants.Drive.FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
  static CANSparkMax rightMotorFollower = new CANSparkMax(Constants.Drive.BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);

  // PID CONTROLLER FOR DRIVER //
   //TODO: TUNE THESE VALUES BEFORE FIRST RUN.
   //! BE CAREFUL WITH THESE VALUES, THEY CAN CAUSE MOTOR WINDUP AND OTHER ISSUES.
   private static final double kP = 0.1;
   private static final double kI = 0.0;
   private static final double kD = 0.0;
   private static final double kF = 0.0;

   private static final double kS = 0.0;  // STATIC Feedforward Gain.
   private static final double kV = 0.0;  // Velocity Feedforward Gain.
   private static final double kA = 0.0;  // Acceleration Feedforward Gain.

	 private double leftSetpoint = 0.0;
   private double rightSetpoint = 0.0;
	 private double leftOutput = 0.0;
	 private double rightOutput = 0.0;

   private SparkPIDController ml_pidController = leftMotorLeader.getPIDController();
   private SparkPIDController mr_pidController = rightMotorLeader.getPIDController();

  // Our actual drive controller.
  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  
  public DriveTrain() {

    /* Setup the followers of the motors. */
    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorLeader);

    //leftLeader.setSmartCurrentLimit();
    //rightLeader.setSmartCurrentLimit();

    /* Sets the proper directions for the motor groups. */
    rightMotorLeader.setInverted(false);
    leftMotorLeader.setInverted(true);

    /* Open loopback code that is commented? */
    //leftLeader.setOpenLoopRampRate(.85);
    //rightLeader.setOpenLoopRampRate(.85);

    leftMotorLeader.setIdleMode(IdleMode.kBrake);
    rightMotorLeader.setIdleMode(IdleMode.kBrake);
    leftMotorFollower.setIdleMode(IdleMode.kBrake);
    rightMotorFollower.setIdleMode(IdleMode.kBrake);

    /* Set open-loop rate for motors. */
    leftMotorFollower.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);
    rightMotorFollower.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);
    leftMotorLeader.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);
    rightMotorLeader.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);

    //Setup PID Values
    ml_pidController.setP(kP);
    ml_pidController.setI(kI);
    ml_pidController.setD(kD);
    ml_pidController.setFF(kF);

    mr_pidController.setP(kP);
    mr_pidController.setI(kI);
    mr_pidController.setD(kD);
    mr_pidController.setFF(kF);

		ml_pidController.setReference(leftSetpoint, ControlType.kVelocity);
	  mr_pidController.setReference(rightSetpoint, ControlType.kVelocity);

    setupDashboard(false);
  }

  void setupDashboard(Boolean update) {
    if (Constants.Debug.debugMode && !update) {
      /* Add PID and feedforward constants to Shuffleboard. */
			SmartDashboard.putNumber("Left Motor P", ml_pidController.getP());
      SmartDashboard.putNumber("Left Motor I", ml_pidController.getI());
      SmartDashboard.putNumber("Left Motor D", ml_pidController.getD());
    	SmartDashboard.putNumber("Left Motor FF", ml_pidController.getFF());
    	SmartDashboard.putNumber("Right Motor P", mr_pidController.getP());
    	SmartDashboard.putNumber("Right Motor I", mr_pidController.getI());
    	SmartDashboard.putNumber("Right Motor D", mr_pidController.getD());
    	SmartDashboard.putNumber("Right Motor FF", mr_pidController.getFF());
    } else {
      /* Update PID and stuff with edited values from the dash. */
      ml_pidController.setP(SmartDashboard.getNumber("Left Motor P", ml_pidController.getP()));
      ml_pidController.setI(SmartDashboard.getNumber("Left Motor I", ml_pidController.getI()));
      ml_pidController.setD(SmartDashboard.getNumber("Left Motor D", ml_pidController.getD()));
      ml_pidController.setFF(SmartDashboard.getNumber("Left Motor FF", ml_pidController.getFF()));
      mr_pidController.setP(SmartDashboard.getNumber("Right Motor P", mr_pidController.getP()));
      mr_pidController.setI(SmartDashboard.getNumber("Right Motor I", mr_pidController.getI()));
      mr_pidController.setD(SmartDashboard.getNumber("Right Motor D", mr_pidController.getD()));
      mr_pidController.setFF(SmartDashboard.getNumber("Right Motor FF", mr_pidController.getFF()));
    }
  }

  /*  Easy way to get instances. Returns the drivetrain obj if it's not null. */
  public static DriveTrain getInstance() {
    if (driveTrain == null) {driveTrain = new DriveTrain();}
    return driveTrain;
  }

  //The arcade drive mode, don't ask why it's swapped.
  public void driveArcade (double forwardBackSpeed, double rotationSpeed) {
    differentialDrive.arcadeDrive(forwardBackSpeed, rotationSpeed/1.25);
  }

  //TODO: find what these values represent
  public double rightDistance() {
    return Units.inchesToMeters((rightMotorLeader.getEncoder().getPosition() * 6 * Math.PI) / 8.01);
  }

  public double leftDistance() {
    return Units.inchesToMeters((leftMotorLeader.getEncoder().getPosition() * 6 * Math.PI) / 8.01);
  }

  public void moveVolts(double leftVoltage, double rightVoltage) {
    leftMotorLeader.setVoltage(leftVoltage);
    rightMotorLeader.setVoltage(rightVoltage);
  }

  /* New PID Code! WIP */
  public void drivePID() {
    // Apply output to motors
    leftMotorLeader.set(leftOutput);
    rightMotorLeader.set(rightOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

		// Get output from PID controllers
    leftOutput = leftMotorLeader.get();
    rightOutput = rightMotorLeader.get();

    /* Update dashboard. */
    setupDashboard(true);
  }
}
