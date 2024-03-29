// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.RobotContainer;

public class HoldArmInPositionCmd extends ProfiledPivotArmCmd {
  double angle;

  /**
   * Holds the arm in position with target angle.
   * @param angleSetpoint The angle to hold the arm at.
   */
  public HoldArmInPositionCmd(double angleSetpoint) {
    super(angleSetpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Holding arm in pos " + System.currentTimeMillis());
    // End when the operator wants to move the arm or until the driver wants to end it.
    return Math.abs(RobotContainer.operatorController.getRightY()) > .1 ||
        RobotContainer.operatorController.a().getAsBoolean() ||
        RobotContainer.operatorController.b().getAsBoolean() ||
        RobotContainer.operatorController.x().getAsBoolean() ||
        RobotContainer.operatorController.y().getAsBoolean() ||
        RobotContainer.driveController.a().getAsBoolean();
  }
}
