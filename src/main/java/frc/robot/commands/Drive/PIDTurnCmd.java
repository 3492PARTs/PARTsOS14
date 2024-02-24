// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.security.cert.TrustAnchor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PIDValues;

public class PIDTurnCmd extends Command {
  /** Creates a new PIDTurn. */
  double initialAngle;
  double setPoint;
  double[] pidValues;
  PIDController rotPIDController;
  DriveTrain driveTrain;

  public PIDTurnCmd(PIDValues values, double setPoint) {
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
    // calculating how much distance covered and how much distance needed to cover
    // to get to setpoint
    double volts = rotPIDController.calculate(driveTrain.getGyroAngle() - initialAngle);
    volts = MathUtil.clamp(volts, -6, 6);

    driveTrain.moveVolts(volts, -volts);
    SmartDashboard.putNumber("PID Turn", volts);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotPIDController.atSetpoint();
  }
}
