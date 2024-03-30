// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class PIDTurnCmd extends Command {
  /** Creates a new PIDTurn. */
  double initialAngle;
  double setPoint;
  PIDController rotPIDController;
  DriveTrain driveTrain;

  public PIDTurnCmd(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = DriveTrain.getInstance();
    rotPIDController = new PIDController(Constants.Drive.turning_kP, Constants.Drive.turning_kI,
        Constants.Drive.turning_kD);
    this.setPoint = setPoint;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroGyro();
    rotPIDController.reset();
    initialAngle = driveTrain.getGyroAngle();

    rotPIDController.setTolerance(4);
    rotPIDController.setSetpoint(setPoint);
    rotPIDController.setIntegratorRange(-1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculating how much distance covered and how much is left.
    double volts = rotPIDController.calculate(driveTrain.getGyroAngle() - initialAngle);
    volts = MathUtil.clamp(volts, -6, 6);

    driveTrain.moveVolts(-volts, volts);
    //SmartDashboard.putNumber("PID Turn", volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.driveTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean motorVelocityUnderThreshold = Math.abs(driveTrain.getMotorVelocity()) < .02;

    if (Constants.Debug.debugMode) {
      SmartDashboard.putBoolean(String.format("%s at setpoint", getName()), rotPIDController.atSetpoint());
      SmartDashboard.putNumber(String.format("%s motor velocity", getName()), Math.abs(driveTrain.getMotorVelocity()));
      SmartDashboard.putBoolean(String.format("%s motor velocity under threshold", getName()),
          motorVelocityUnderThreshold);
    }

    return rotPIDController.atSetpoint() && motorVelocityUnderThreshold;
  }
}
