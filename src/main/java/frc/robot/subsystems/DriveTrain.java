// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
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

  // PID CONTROLLER FOR DRIVER //
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private SparkPIDController ml_pidController = leftMotorLeader.getPIDController();
  private SparkPIDController mr_pidController = rightMotorLeader.getPIDController();
  private RelativeEncoder m_l_encoder;
  private RelativeEncoder m_r_encoder;

  // Our actual drive controller.
  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);
  
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
    //rightLeader.setOpenLoopRampRate

    leftMotorLeader.setIdleMode(IdleMode.kBrake);
    rightMotorLeader.setIdleMode(IdleMode.kBrake);
    leftMotorFollower.setIdleMode(IdleMode.kBrake);
    rightMotorFollower.setIdleMode(IdleMode.kBrake);

    //Encoder object created to display position values
    m_l_encoder = rightMotorLeader.getEncoder();
    m_r_encoder = leftMotorLeader.getEncoder();

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    ml_pidController.setP(kP);
    ml_pidController.setI(kI);
    ml_pidController.setD(kD);
    ml_pidController.setIZone(kIz);
    ml_pidController.setFF(kFF);
    ml_pidController.setOutputRange(kMinOutput, kMaxOutput);

    mr_pidController.setP(kP);
    mr_pidController.setI(kI);
    mr_pidController.setD(kD);
    mr_pidController.setIZone(kIz);
    mr_pidController.setFF(kFF);
    mr_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  // Easy way to get instances.
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
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { ml_pidController.setP(p); kP = p; }
    if((i != kI)) { ml_pidController.setI(i); kI = i; }
    if((d != kD)) { ml_pidController.setD(d); kD = d; }
    if((iz != kIz)) { ml_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { ml_pidController.setFF(ff); kFF = ff; }

    if((p != kP)) { mr_pidController.setP(p); kP = p; }
    if((i != kI)) { mr_pidController.setI(i); kI = i; }
    if((d != kD)) { mr_pidController.setD(d); kD = d; }
    if((iz != kIz)) { mr_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { mr_pidController.setFF(ff); kFF = ff; }

    if((max != kMaxOutput) || (min != kMinOutput)) { 
      ml_pidController.setOutputRange(min, max);
      mr_pidController.setOutputRange(min, max);  
      kMinOutput = min; kMaxOutput = max; 
    }
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    ml_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    mr_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable Left", m_l_encoder.getPosition());
    SmartDashboard.putNumber("ProcessVariable Right", m_r_encoder.getPosition());
  }
}
