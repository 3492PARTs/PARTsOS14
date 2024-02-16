// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

    // Set open-loop rate for motors.
    leftMotorFollower.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);
    rightMotorFollower.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);
    leftMotorLeader.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);
    rightMotorLeader.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RATE);
  }

  // Easy way to get instances.
  public static DriveTrain getInstance() {
    if (driveTrain == null) {driveTrain = new DriveTrain();}
    return driveTrain;
  }

  public void driveArcade (double forwardBackSpeed, double rotationSpeed) {
    differentialDrive.arcadeDrive(forwardBackSpeed, rotationSpeed/1.25);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
