// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class BangBangShooterCmd extends Command {
  private Shooter shooter;
  private double setpoint;
  private BangBangController bbController = new BangBangController(Constants.Shooter.TOLERANCE);
  private long time = 0;

  public void runShooterBB(double setpoint) {
    // BB calcs 0...1 speed, limiter caps it at 75% for now. Change in Constants if needed.
    Shooter.shooterLeftMotor.set(ControlMode.PercentOutput, calcBB(setpoint));
    Shooter.shooterRightMotor.set(ControlMode.PercentOutput, calcBB(setpoint));
  }

  double calcBB(double setpoint) {
    if (bbController.calculate(Shooter.getInstance().getShooterRPM(), setpoint) != 0) {
      return bbController.calculate(Shooter.getInstance().getShooterRPM(), setpoint) - Constants.Shooter.LIMITER;
    } else {
      // Would be zero anyways, so save the cycle.
      return 0.0;
    }
  }

  /** Creates a new BangBang. */
  public BangBangShooterCmd(double setpoint) {
    this.shooter = Shooter.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.shooterLeftMotor.setNeutralMode(NeutralMode.Coast);
    Shooter.shooterRightMotor.setNeutralMode(NeutralMode.Coast);
    time = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runShooterBB(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop dem motor !!!
    Shooter.shooterLeftMotor.set(ControlMode.PercentOutput, 0);
    Shooter.shooterRightMotor.set(ControlMode.PercentOutput, 0);
    runShooterBB(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // the time part is to keep from false stopping when the command starts
    return System.currentTimeMillis() - time > 200 &&
        (RobotContainer.operatorController.leftBumper().getAsBoolean() ||
            RobotContainer.operatorController.leftTrigger().getAsBoolean() ||
            RobotContainer.operatorController.a().getAsBoolean() ||
            RobotContainer.operatorController.b().getAsBoolean() ||
            RobotContainer.operatorController.x().getAsBoolean() ||
            RobotContainer.operatorController.y().getAsBoolean() ||
            RobotContainer.operatorController.povUp().getAsBoolean());
  }
}
