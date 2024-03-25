// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class RunIntakePhotoEyeCmd extends Command {
  /** Creates a new RunIntakePhotoEyeCommand. */
  double speed;
  Intake intake;
  long startTime = 0;

  public RunIntakePhotoEyeCmd(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = Intake.getInstance();
    this.speed = speed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(speed);

    if (!intake.hasNote())
      this.startTime = System.currentTimeMillis();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Must sense note for at least 200ms or operator interrupt
    return System.currentTimeMillis() - this.startTime >= 200 || RobotContainer.operatorInterrupt();
  }
}
