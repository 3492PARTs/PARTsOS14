// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmToPositionCmd extends Command {
  /** Creates a new armToPosition. */
  Arm arm;
  double angle;
  boolean direction;

  public ArmToPositionCmd(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: check to see if the parameter is in rotations or degrees
    arm.setPivotPoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    direction = arm.getAngle() > angle;

    //TODO: use slower speeds
    if (direction) {
      arm.setPivotSpeed(-0.2);
    }
    else {
      arm.setPivotSpeed(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: may need to add rotation rate to return statement
    //return (Math.abs(angle - Elevator.getInstance().getAngle()) < 3) && Elevator.getInstance().getRotationRate() < 30;
    return (Math.abs(angle - arm.getAngle()) < 3);
  }
}

