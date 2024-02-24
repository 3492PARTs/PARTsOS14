// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class PhotoEyeArmUpCmd extends Command {
  /** Creates a new PhotoEyeArmUpCmd. */
  Arm arm;
  Intake intake;

  public PhotoEyeArmUpCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    intake = Intake.getInstance();
    addRequirements(intake);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: get angle to go to ground
    new ProfiledPivotArm(80, 2.7, 0, 0).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
