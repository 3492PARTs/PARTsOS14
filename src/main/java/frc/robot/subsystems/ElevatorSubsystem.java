// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  static CANSparkMax pivotMotorLeader;
  static CANSparkMax pivotMotorFollower;

  public ElevatorSubsystem() {

    pivotMotorLeader = new CANSparkMax(Constants.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotorFollower = new CANSparkMax(Constants.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);
  }

  public double getAngle() {
    
  }

  public void setPivotSpeed(double speed) {

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
