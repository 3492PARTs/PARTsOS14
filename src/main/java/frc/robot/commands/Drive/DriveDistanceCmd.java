// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveDistanceCmd extends Command {
  double distance; // meters
  DriveTrain driveTrain;
  boolean direction;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveDistanceCmd(double distance) {
    driveTrain = DriveTrain.getInstance();
    this.distance = distance;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    direction = ((driveTrain.leftDistance() + driveTrain.rightDistance()) / 2) > distance;

    // if distance is greater than set point, then drive backwards, otherwise drive
    // forward
    if (direction) {
      driveTrain.driveArcade(-0.5, 0);
    } else {
      driveTrain.driveArcade(0.5, 0);
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
    // distance is in meters
    return Math.abs(distance - ((driveTrain.leftDistance() + driveTrain.rightDistance()) / 2)) < .2;
  }
}
