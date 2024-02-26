// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveAngleCmd extends Command {
  double angle; // meters
  DriveTrain driveTrain;
  boolean direction;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveAngleCmd(double angle) {
    driveTrain = DriveTrain.getInstance();
    this.angle = angle;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    direction = driveTrain.getGyroAngle() > angle;

    // if angle is greater than set point, then drive backwards, otherwise drive
    // forward
    if (direction) {
      driveTrain.driveArcade(0, -0.5);
    } else {
      driveTrain.driveArcade(0, 0.5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // angle is in meters
    System.out.println(driveTrain.getGyroAngle());
    return Math.abs(angle) - Math.abs(driveTrain.getGyroAngle()) < .3;
  }
}
