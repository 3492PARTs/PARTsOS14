// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

  // Encoder shooterEncoder;

  public Shooter() {
    // Intialize the motors.
    shooterLeftLeader = new TalonSRX(Constants.Shooter.LEFT_SHOOTER_MOTOR);
    shooterRightFollower = new TalonSRX(Constants.Shooter.RIGHT_SHOOTER_MOTOR);

    shooterRightFollower.follow(shooterLeftLeader);

    shooterRightFollower.setInverted(false);
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