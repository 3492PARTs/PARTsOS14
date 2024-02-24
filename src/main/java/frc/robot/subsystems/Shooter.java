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
  final double shooterWheelRadius = 3.98;

  WPI_CANCoder shooterLeftEncoder;
  WPI_CANCoder shooterRightEncoder;

  public Shooter() {
    // Intialize the motors.
    shooterLeftLeader = new TalonSRX(Constants.Shooter.LEFT_SHOOTER_MOTOR);
    shooterRightFollower = new TalonSRX(Constants.Shooter.RIGHT_SHOOTER_MOTOR);

    shooterRightFollower.follow(shooterLeftLeader);

    shooterRightFollower.setInverted(false);
    shooterLeftLeader.setInverted(true);

    shooterLeftEncoder = new WPI_CANCoder(Constants.Shooter.LEFT_SHOOTER_MOTOR);
    shooterRightEncoder = new WPI_CANCoder(Constants.Shooter.RIGHT_SHOOTER_MOTOR);
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
  }

  public double getAverageShooterVelocity() {
    double average = (shooterRightEncoder.getVelocity() + shooterLeftEncoder.getVelocity()) / 2;
    return average;
  }

  public double getAverageShooterPosition() {
    double average = (shooterLeftEncoder.getPosition() + shooterRightEncoder.getPosition()) / 2;
    return average;
  }
  /*
   * public double countsToMeters (double counts) {
   * double motorRotations = (double) counts/countsPerRev;
   * double wheelRotations = motorRotations/shooterGearRatio;
   * double positionInMeters = wheelRotations * (2 * Math.PI *
   * Units.inchesToMeters(shooterWheelRadius));
   * return positionInMeters;
   * }
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}