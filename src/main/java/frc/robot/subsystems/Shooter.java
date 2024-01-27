// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter shooterInstance;
  //private final

  /** Creates a new Shooter. */
  public static Shooter getInstance() {
    // If instance is null, then make a new instance.
    if (shooterInstance == null) { shooterInstance = new Shooter(); }
    return shooterInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
