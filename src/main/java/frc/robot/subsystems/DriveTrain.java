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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

    if (Constants.Debug.debugMode) {
      /* Add PID and feedforward constants to Shuffleboard. */
			SmartDashboard.putNumber("Left Motor P", ml_pidController.getP());
      SmartDashboard.putNumber("Left Motor I", ml_pidController.getI());
      SmartDashboard.putNumber("Left Motor D", ml_pidController.getD());
    	SmartDashboard.putNumber("Left Motor FF", ml_pidController.getFF());
    	SmartDashboard.putNumber("Right Motor P", mr_pidController.getP());
    	SmartDashboard.putNumber("Right Motor I", mr_pidController.getI());
    	SmartDashboard.putNumber("Right Motor D", mr_pidController.getD());
    	SmartDashboard.putNumber("Right Motor FF", mr_pidController.getFF());
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

		// Get output from PID controllers
    leftOutput = leftMotorLeader.get();
    rightOutput = rightMotorLeader.get();

		// Apply output to motors
    leftMotorLeader.set(leftOutput);
    rightMotorLeader.set(rightOutput);
  }
}
