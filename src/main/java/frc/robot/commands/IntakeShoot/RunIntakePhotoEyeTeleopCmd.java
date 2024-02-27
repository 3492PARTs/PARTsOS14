// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeShoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.subsystems.Intake;

public class RunIntakePhotoEyeTeleopCmd extends Command {
  /** Creates a new RunIntakePhotoEyeCommand. */
  double speed;
  Intake intake;
  double armPosition;

  public RunIntakePhotoEyeTeleopCmd(double speed, double armPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = Intake.getInstance();
    this.speed = speed;
    this.armPosition = armPosition;
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
    if (intake.hasNote())
      new ArmToPositionTeleopCmd(armPosition).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote() ||
        RobotContainer.operatorController.leftBumper().getAsBoolean() ||
        RobotContainer.operatorController.leftTrigger().getAsBoolean() ||
        RobotContainer.operatorController.a().getAsBoolean() ||
        RobotContainer.operatorController.b().getAsBoolean() ||
        RobotContainer.operatorController.y().getAsBoolean();
  }
}
