// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class HoldArmInPositionCmd extends ProfiledPivotArmCmd {
  double angle;

  /**
   * Holds the arm in position with target angle.
   * @param angle The angle to hold the arm at.
   */
  public HoldArmInPositionCmd(double angle) {
    super(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when the operator wants to move the arm or until the driver wants to end it.
    return Math.abs(RobotContainer.operatorController.getRightY()) > .1 ||
        RobotContainer.operatorController.a().getAsBoolean() ||
        RobotContainer.operatorController.b().getAsBoolean() ||
        RobotContainer.operatorController.x().getAsBoolean() ||
        RobotContainer.operatorController.y().getAsBoolean() ||
        RobotContainer.driveController.a().getAsBoolean();
  }
}
