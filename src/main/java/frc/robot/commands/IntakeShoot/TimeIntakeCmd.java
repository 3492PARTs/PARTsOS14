// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeShoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class TimeIntakeCmd extends Command {
  /** Creates a new Intake. */
  // Make sure to initalize these to avoid weird Java shananigans.
  Intake intake;
  long startTime = 0;
  double duration = 0.0;

  public TimeIntakeCmd(double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.duration = seconds;
    this.intake = Intake.getInstance();
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(Constants.Intake.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= duration * 1000;
  }
}
