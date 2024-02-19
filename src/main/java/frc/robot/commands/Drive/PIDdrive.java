// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PIDValues;

public class PIDdrive extends Command {
  double initialPos;
  double setPoint;
  DriveTrain driveTrain;
  double [] pidValues;
  //PIDController drivePIDController;

  /** Creates a new PIDdrive. */
  public PIDdrive( double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    //this.pidValues = pidValues.getPIDValues();
    //drivePIDController = new PIDController (pidValues[0], pidValues[1], pidValues[2]);
    this.setPoint = setPoint;
    this.driveTrain = DriveTrain.getInstance();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //calculates average distance
    initialPos = (driveTrain.leftDistance() + driveTrain.rightDistance()) /2;

    driveTrain.drivePIDController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = driveTrain.drivePIDController.calculate(((driveTrain.leftDistance() + driveTrain.rightDistance()) /2) - initialPos);

    speed = MathUtil.clamp(speed, -7, 7);

    driveTrain.moveVolts(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveTank(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.drivePIDController.atSetpoint();
  }
}
