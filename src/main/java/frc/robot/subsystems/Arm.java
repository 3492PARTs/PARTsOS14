// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO: implement PID values
public class Arm extends SubsystemBase {

  private static Arm armInstance;
  /** Creates a new ArmSubsystem. */

  static CANSparkMax pivotLeftMotor;
  static CANSparkMax pivotRightMotor;
  
  SparkPIDController pivotLeftController;
  SparkPIDController pivotRightController;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  //TODO: update gear ratio 
  double pivotGearRatio = Constants.Arm.PIVOT_GEAR_RATIO;

  public Arm() {
    pivotLeftMotor= new CANSparkMax(Constants.Arm.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(Constants.Arm.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

    pivotLeftMotor.setInverted(true);
    pivotRightMotor.setInverted(false);

    pivotLeftController = pivotLeftMotor.getPIDController();
    pivotRightController = pivotRightMotor.getPIDController();

    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

    pivotLeftMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);
    pivotRightMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);

    Shuffleboard.getTab("debug").addNumber("arm angle", getAngleSupplier());
    Shuffleboard.getTab("debug").addNumber("arm angular velocity", getAnglularVelocitySupplier());

  }

  public static Arm getInstance() {
    // If instance is null, then make a new instance.
    if (armInstance == null) { armInstance = new Arm(); }
    return armInstance;
  }


  public double getAngle() {
    return 360 * pivotLeftMotor.getEncoder().getPosition() / pivotGearRatio;
  }

  public DoubleSupplier getAngleSupplier() {
    DoubleSupplier s = () -> getAngle();
    return s;
  }


  //TODO: check number 60
  public double getRotationRate() {
    return 360 * pivotLeftMotor.getEncoder().getVelocity() / (pivotGearRatio * 60);

  }

  public DoubleSupplier getAnglularVelocitySupplier() {
    DoubleSupplier s = () -> getRotationRate();
    return s;
  }


  public void setPivotSpeed(double speed) {
    pivotLeftMotor.set(speed/2);
    pivotRightMotor.set(speed/2);
  }

  //TODO: check to see if position is in rotations or degrees
  public void setPivotPoint(double position) {
    pivotLeftController.setReference(position, ControlType.kPosition);
    pivotRightController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Shuffleboard.update();
  }
}
