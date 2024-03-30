// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Logger;

public class ShootCmd extends Command {
  /** Creates a new Shoot. */
  Shooter shooter;

  double speed = 0;

  public ShootCmd(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // shooterEncoder = new Encoder()
    this.shooter = Shooter.getInstance();
    this.speed = speed;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.getInstance().logString(this.getName(), "start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().logString(this.getName(), String.format("end, interrupted: %s", interrupted));
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
