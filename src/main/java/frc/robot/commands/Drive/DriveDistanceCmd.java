// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveDistanceCmd extends Command {
  double distance; // meters
  DriveTrain driveTrain;
  boolean direction;

  public DriveDistanceCmd(double distance) {
    driveTrain = DriveTrain.getInstance();
    this.distance = distance;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.getInstance().logString(this.getName(), "start");
    driveTrain.zeroDriveEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    direction = ((driveTrain.leftDistance() + driveTrain.rightDistance()) / 2) > distance;

    // If angle is greater than set point, drive backwards, otherwise drive forward.
    if (direction) {
      driveTrain.driveArcade(0.5, 0);
    } else {
      driveTrain.driveArcade(-0.5, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().logString(this.getName(), String.format("end, interrupted: %s", interrupted));
    driveTrain.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // distance is in meters
    // .025 m -> 1 in
    // .13 m -> 5 in
    return Math.abs(distance - ((driveTrain.leftDistance() + driveTrain.rightDistance()) / 2)) < .025; //.13;
  }
}
