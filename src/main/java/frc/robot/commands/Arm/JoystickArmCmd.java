// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class JoystickArmCmd extends Command {
  /** Creates a new JoystickArmCommand. */
  private CommandXboxController operatorController;
  private Arm arm;

  public JoystickArmCmd(CommandXboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.operatorController = operatorController;
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Manual control with a lower hard stop.
    //at bottom limit
    if (arm.getLimitSwitch()) {
      // Negative controller value is up on arm
      if (operatorController.getRightY() < 0)
        arm.setSpeed(operatorController.getRightY());
      else
        arm.setSpeed(0);
    }
    // at top limit
    else if (arm.getAngle() >= Constants.Arm.UPPER_BOUND) {
      // Positive controller value is down on arm
      if (operatorController.getRightY() > 0)
        arm.setSpeed(operatorController.getRightY());
      else
        arm.setSpeed(0);
    }
    // full manual, in safe bounds
    else
      arm.setSpeed(operatorController.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(operatorController.getRightY()) <= Constants.Arm.JOYSTICK_CONTROL_LIMIT;
  }
}
