// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PIDValues;

public class PIDdrive extends Command {
  double initialPos;
  double setPoint;
  DriveTrain driveTrain;
  double[] pidValues;
  PIDController drivePIDController;

  /** Creates a new PIDdrive. */
  public PIDdrive(PIDValues pValues, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pidValues = pValues.getPIDValues();
    drivePIDController = new PIDController(pidValues[0], pidValues[1], pidValues[2]);
    this.setPoint = setPoint;
    this.driveTrain = DriveTrain.getInstance();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // calculates average distance
    // initialPos = (driveTrain.leftDistance() + driveTrain.rightDistance()) /2;

    drivePIDController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = .2 + drivePIDController.calculate((driveTrain.leftDistance() + driveTrain.rightDistance()) / 2);

    speed = MathUtil.clamp(speed, -.5, .5);

    driveTrain.driveArcade(-speed, 0);
    SmartDashboard.putNumber("drive setPoint", drivePIDController.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePIDController.atSetpoint();
  }
}
