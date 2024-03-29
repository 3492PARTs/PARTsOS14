// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.subsystems.Intake;

/**
 * IntakePhotoEyeArmPosCmd
 *
 * @deprecated  Should not call schedule in a command. Use sequences.
 * private method.
 * @see #IntakeArmPositionCmdSeq
 */
@Deprecated
public class IntakePhotoEyeArmPosCmd extends RunIntakePhotoEyeCmd {
  /** Creates a new RunIntakePhotoEyeCommand. */
  double speed;
  Intake intake;
  double armPosition;
  long startTime = 0;

  public IntakePhotoEyeArmPosCmd(double speed, double armPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(speed);
    this.speed = speed;
    this.intake = Intake.getInstance();
    this.armPosition = armPosition;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
    if (intake.hasNote()) {
      new TimeIntakeCmd(.2, .38).schedule();
      new ProfiledPivotArmCmd(armPosition).schedule();
    }
  }
}
