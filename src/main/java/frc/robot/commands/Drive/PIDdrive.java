// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class PIDdrive extends Command {

  double intialPosition;
  double setPoint;
  DriveTrain driveTrain;
  PIDController PIDController;

  //TODO: tune values
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;

  /** Creates a new PIDdrive. */
  public PIDdrive(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setPoint = setPoint;
    this.driveTrain = DriveTrain.getInstance();
    PIDController = new PIDController(kP, kI, kD);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //calculates average distance
    intialPosition = (driveTrain.leftDistance() + driveTrain.rightDistance()) / 2;
    PIDController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = PIDController.calculate(((driveTrain.leftDistance() + driveTrain.rightDistance()) /2) - intialPosition);
    speed = MathUtil.clamp(speed, -12, 12);
    driveTrain.moveVolts(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveArcade(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PIDController.atSetpoint();
  }
}
