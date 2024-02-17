// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

   //TODO: TUNE THESE VALUES BEFORE FIRST RUN.
   private static final double kP = 0.1;
   private static final double kI = 0.0;
   private static final double kD = 0.0;

   public PIDController drivePIDController;
   //public PIDController leftPIDController;
   //public PIDController rightPIDController;

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

  /* feedforward
  private static final double kS = 0.0;  // STATIC Feedforward Gain.
  private static final double kV = 0.0;  // Velocity Feedforward Gain.
  private static final double kA = 0.0;  // Acceleration Feedforward Gain.
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  */
  
  public DriveTrain() {

    /* Setup the followers of the motors. */
    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorLeader);

    /* Sets the proper directions for the motor groups. */
    rightMotorLeader.setInverted(false);
    leftMotorLeader.setInverted(true);

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
    drivePIDController = new PIDController(kP, kI, kD);

    setupDashboard(false);

    /* 
    leftPIDController = new PIDController(kP, kI, kD);
    rightPIDController = new PIDController(kP, kI, kD);
    
    leftPIDController.setP(kP);
    leftPIDController.setI(kI);
    leftPIDController.setD(kD);
    leftPIDController.setFF(kF);

    rightPIDController.setP(kP);
    rightPIDController.setI(kI);
    rightPIDController.setD(kD);
    rightPIDController.setFF(kF);
    */
  }

  void setupDashboard(Boolean update) {
    if (Constants.Debug.debugMode && !update) {
      /* Add PID and feedforward constants to Shuffleboard. */
			SmartDashboard.putNumber("Motor kP", drivePIDController.getP());
      SmartDashboard.putNumber("Motor kI", drivePIDController.getI());
      SmartDashboard.putNumber("Motor kD", drivePIDController.getD());
    	  //SmartDashboard.putNumber("Left Motor FF", leftPIDController.getFF());
    	  //SmartDashboard.putNumber("Right Motor P", rightPIDController.getP());
    	  //SmartDashboard.putNumber("Right Motor I", rightPIDController.getI());
    	  //SmartDashboard.putNumber("Right Motor D", rightPIDController.getD());
        //SmartDashboard.putNumber("Right Motor FF", rightPIDController.getFF());
    SmartDashboard.updateValues();
    } else {
      /* Update PID and stuff with edited values from the dash. */
      drivePIDController.setP(SmartDashboard.getNumber("Left Motor P", drivePIDController.getP()));
      drivePIDController.setI(SmartDashboard.getNumber("Left Motor I", drivePIDController.getI()));
      drivePIDController.setD(SmartDashboard.getNumber("Left Motor D", drivePIDController.getD()));
        //leftPIDController.setFF(SmartDashboard.getNumber("Left Motor FF", leftPIDController.getFF()));
        //rightPIDController.setP(SmartDashboard.getNumber("Right Motor P", rightPIDController.getP()));
        //rightPIDController.setI(SmartDashboard.getNumber("Right Motor I", rightPIDController.getI()));
        //rightPIDController.setD(SmartDashboard.getNumber("Right Motor D", rightPIDController.getD()));
        // rightPIDController.setFF(SmartDashboard.getNumber("Right Motor FF", rightPIDController.getFF()));
     SmartDashboard.updateValues();
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

  public void driveTank(double leftMotorSpeed, double rightMotorSpeed){
    differentialDrive.tankDrive(leftMotorSpeed, rightMotorSpeed);
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

  public void zeroDriveEncoders() {
    leftMotorLeader.getEncoder().setPosition(0);
    rightMotorLeader.getEncoder().setPosition(0);
  }

  /* 
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    SmartDashboard.putNumber("left wheel speed ", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right wheel speed ", speeds.rightMetersPerSecond);

    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    SmartDashboard.putNumber("left feed forward wheel speed ", leftFeedforward);
    SmartDashboard.putNumber("right reed forward wheel speed ", rightFeedforward);

    final double leftOutput =
        leftPIDController.calculate(leftMotorLeader.getEncoder().getPosition(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(rightMotorLeader.getEncoder().getPosition(), speeds.rightMetersPerSecond);

    SmartDashboard.putNumber("left pid wheel speed ", leftOutput);
    SmartDashboard.putNumber("right pid wheel speed", rightOutput);

    leftMotorLeader.setVoltage(leftOutput + leftFeedforward);
    rightMotorLeader.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    SmartDashboard.putNumber("speed ", xSpeed);
    SmartDashboard.putNumber(" rot ", rot);
    xSpeed = xSpeed * Constants.Drive.MAX_SPEED;
    rot = rot * Constants.Drive.MAX_ANGULAR_SPEED;
    var wheelSpeeds = dKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void drivePID() {
    // Apply output to motors
    leftMotorLeader.set(leftOutput);
    rightMotorLeader.set(rightOutput);
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Update dashboard. */
    setupDashboard(true);
  }
}
