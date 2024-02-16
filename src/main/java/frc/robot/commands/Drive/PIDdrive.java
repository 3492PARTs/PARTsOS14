// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class PIDdrive extends Command {

  double setPoint;
  DriveTrain driveTrain;

  /** Creates a new PIDdrive. */
  public PIDdrive(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setPoint = setPoint;
    this.driveTrain = DriveTrain.getInstance();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //calculates average distance
    driveTrain.leftPIDController.setSetpoint(setPoint);
    driveTrain.rightPIDController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(.5,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.leftPIDController.atSetpoint() && driveTrain.rightPIDController.atSetpoint();
  }
}
