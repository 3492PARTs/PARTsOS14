// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

@Deprecated
public class ArmToPositionAutoCmd extends Command {
  /** Creates a new armToPosition. */
  Arm arm;
  double angle;
  boolean direction;

  public ArmToPositionAutoCmd(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    direction = arm.getAngle() > angle;

    if (direction) {
      arm.setSpeed(-0.18);
    } else {
      arm.setSpeed(0.18);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(angle - arm.getAngle()) < .2);
  }
}
