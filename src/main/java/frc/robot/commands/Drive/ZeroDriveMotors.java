// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class ZeroDriveMotors extends Command {
  DriveTrain driveTrain;

  /** Creates a new ZeroDriveMotors. */
  public ZeroDriveMotors() {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = DriveTrain.getInstance();
    addRequirements(driveTrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
