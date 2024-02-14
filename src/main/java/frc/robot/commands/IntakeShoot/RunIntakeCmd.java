// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeShoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeCmd extends Command {
  /** Creates a new RunIntakeCmd. */
  int direction;
  Intake intake;
  public RunIntakeCmd(int direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = Intake.getInstance();
    this.direction = direction;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: add intake.hasNote() in parameters when adding in photoEye
    intake.runIntake(direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
