// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private static Shooter shooterInstance;

  /** Creates a new Shooter. */
  public static TalonSRX shooterLeftLeader;
  static TalonSRX shooterRightFollower;
  public final int countsPerRev = 1024;
  // final double shooterGearRatio;
  final double shooterWheelRadius = 2.0;

  public Shooter() {
    // Intialize the motors.
    shooterLeftLeader = new TalonSRX(Constants.Shooter.LEFT_MOTOR);
    shooterRightFollower = new TalonSRX(Constants.Shooter.RIGHT_MOTOR);

    shooterRightFollower.setInverted(true);
    shooterLeftLeader.setInverted(true);
  }

  public static Shooter getInstance() {
    // If instance is null, then make a new instance.
    if (shooterInstance == null) {
      shooterInstance = new Shooter();
    }
    return shooterInstance;
  }

  public void runShooter(double speed) {
    shooterLeftLeader.set(ControlMode.PercentOutput, speed);
    shooterRightFollower.set(ControlMode.PercentOutput, speed);
  }

  public double getShooterRPM() {
    double RPM = -(((shooterRightFollower.getSelectedSensorVelocity() + shooterLeftLeader.getSelectedSensorVelocity())
        / 2)
        * (10.0 / 4096) * (2 * Math.PI * shooterWheelRadius));
    // double average = (shooterRightEncoder.getVelocity() +
    // shooterLeftEncoder.getVelocity()) / 2;
    return RPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}