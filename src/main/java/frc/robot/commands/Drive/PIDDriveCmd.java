// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Logger;

public class PIDDriveCmd extends Command {
  double initialPos;
  double setPoint;
  DriveTrain driveTrain;
  PIDController drivePIDController;

  /** Creates a new PIDdrive. */
  public PIDDriveCmd(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivePIDController = new PIDController(Constants.Drive.kP, Constants.Drive.kI, Constants.Drive.kD);
    this.setPoint = setPoint;
    this.driveTrain = DriveTrain.getInstance();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.getInstance().logString(this.getName(), "start");
    driveTrain.zeroDriveEncoders();
    //drivePIDController.reset(); // Remove if this causes errors
    // Calculates average distance.
    initialPos = (driveTrain.leftDistance() + driveTrain.rightDistance()) / 2;

    drivePIDController.setTolerance(.05);
    drivePIDController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculating how much distance covered and how much is left.
    double volts = -drivePIDController
        .calculate(((driveTrain.leftDistance() + driveTrain.rightDistance()) / 2) - initialPos);

    volts = MathUtil.clamp(volts, -10, 10);

    driveTrain.moveVolts(volts, volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().logString(this.getName(), String.format("end, interrupted: %s", interrupted));
    driveTrain.driveTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePIDController.atSetpoint() && Math.abs(driveTrain.getMotorVelocity()) < .01;
  }
}
