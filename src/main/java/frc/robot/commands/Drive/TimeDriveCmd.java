// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
@Deprecated
public class TimeDriveCmd extends Command {
  double duration;
  DriveTrain driveTrain;
  long startTime;

  /**
   * Creates a new TimeDriveCmd and drives for the specified duration.
   *
   * @param duration The time to drive in seconds.
   */
  public TimeDriveCmd(double duration) {
    driveTrain = DriveTrain.getInstance();
    this.duration = duration;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.getInstance().logString(this.getName(), "start");
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveArcade(-0.5, 0);
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
    // True if (current time - start time) is greater than or equal to the duration.
    return System.currentTimeMillis() - startTime >= duration * 1000;
  }
}
