// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeShoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmToPositionCmd;
import frc.robot.subsystems.Intake;

public class RunIntakePhotoEyeCommand extends Command {
  /** Creates a new RunIntakePhotoEyeCommand. */
  double speed;
  Intake intake;
  RobotContainer m_robotContainer;

  public RunIntakePhotoEyeCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = Intake.getInstance();
    this.speed = speed;
    addRequirements(intake);
    m_robotContainer = new RobotContainer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
    if (intake.hasNote())
      new ArmToPositionCmd(Constants.Arm.HOME).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote() ||
        m_robotContainer.getOperatorController().leftBumper().getAsBoolean() ||
        m_robotContainer.getOperatorController().leftTrigger().getAsBoolean();
  }
}
