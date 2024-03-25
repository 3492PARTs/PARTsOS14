// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakePhotoEyeArmPosCmd;
import frc.robot.subsystems.Arm;

@Deprecated
public class ArmToPositionTeleopCmd extends Command {
  /** Creates a new armToPosition. */
  Arm arm;
  double angle;
  boolean direction;

  public ArmToPositionTeleopCmd(double angle) {
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
    // if the arm is LOWER than the wanted angle.
    direction = arm.getAngle() > angle;

    if (angle == Constants.Arm.GROUND) {
      Arm.downSpeed = -.35;
    }

    // If above is true, go back up to match set pos. Otherwise continue.
    if (direction) {
      arm.setSpeed(-0.25);
    } else {
      arm.setSpeed(0.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
    // Once arm is on the ground, run the intake to pick up a note.
    if (angle == Constants.Arm.GROUND) {
      new IntakePhotoEyeArmPosCmd(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(angle - arm.getAngle()) < .5);
  }
}
