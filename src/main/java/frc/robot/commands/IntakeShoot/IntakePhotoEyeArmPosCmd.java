// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeShoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.subsystems.Intake;

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
