// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private static CANSparkMax leftClimberMotor;
  private static CANSparkMax rightClimberMotor;

  private static Climber climberInstance;

  public Climber() {
    leftClimberMotor = new CANSparkMax(Constants.Climber.LEFT_MOTOR, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.Climber.RIGHT_MOTOR, MotorType.kBrushless);

    leftClimberMotor.setInverted(true);
    rightClimberMotor.setInverted(false);

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
  }

  public static Climber getInstance() {
    // If instance is null, then make a new instance.
    if (climberInstance == null) {
      climberInstance = new Climber();
    }
    return climberInstance;
  }

  public void setRightSpeed(double speed) {
    rightClimberMotor.set(speed);
  }

  public void setLeftSpeed(double speed) {
    leftClimberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
