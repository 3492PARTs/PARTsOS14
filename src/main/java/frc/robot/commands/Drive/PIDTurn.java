// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PIDValues;

public class PIDTurn extends Command {
  /** Creates a new PIDTurn. */
  double initialAngle;
  double setPoint;
  double [] pidValues;
  PIDController rotPIDController;
  DriveTrain driveTrain;

  public PIDTurn(PIDValues values, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = DriveTrain.getInstance();
    this.pidValues = values.getPIDValues();
    rotPIDController = new PIDController(pidValues[0], pidValues[1], pidValues[2]);
    this.setPoint = setPoint;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = driveTrain.getGyroAngle();
    rotPIDController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = rotPIDController.calculate(driveTrain.getGyroAngle() - initialAngle);
    speed = MathUtil.clamp(speed, -1, 1);
    driveTrain.driveTank(speed, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveTank(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotPIDController.atSetpoint();
  }
}
