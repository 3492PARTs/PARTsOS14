// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private static Shooter shooterInstance;

  /** Creates a new Shooter. */
  public static TalonSRX shooterLeftMotor;
  static TalonSRX shooterRightMotor;
  public final int countsPerRev = 1024;
  final double shooterWheelRadius = 2.0;

  public Shooter() {
    // Initialize the motors.
    shooterLeftMotor = new TalonSRX(Constants.Shooter.LEFT_MOTOR);
    shooterRightMotor = new TalonSRX(Constants.Shooter.RIGHT_MOTOR);

    shooterRightMotor.setInverted(true);
    shooterLeftMotor.setInverted(true);

    shooterRightMotor.configOpenloopRamp(1, 0);
    shooterLeftMotor.configOpenloopRamp(1, 0);

    shooterLeftMotor.configContinuousCurrentLimit(50, 500);
    shooterRightMotor.configContinuousCurrentLimit(50, 500);
  }

  public static Shooter getInstance() {
    // If instance is null, then make a new instance.
    if (shooterInstance == null) {
      shooterInstance = new Shooter();
    }
    return shooterInstance;
  }

  public void runShooter(double speed) {
    shooterLeftMotor.set(ControlMode.PercentOutput, speed);
    shooterRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getShooterRPM() {
    double RPM = -(((shooterRightMotor.getSelectedSensorVelocity() + shooterLeftMotor.getSelectedSensorVelocity()) / 2)
        * (10.0 / 4096) * (2 * Math.PI * shooterWheelRadius));
    return RPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}