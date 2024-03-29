// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

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
    double speed = calcBB(setpoint);
    shooter.runShooter(speed);
  }

  double calcBB(double setpoint) {
    // BB calcs 0...1 speed, limiter caps it at 75% for now. Change in Constants if needed.
    double speed = bbController.calculate(shooter.getShooterRPM(), setpoint);
    return speed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop dem motor !!!
    shooter.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // the time part is to keep from false stopping when the command starts
    return System.currentTimeMillis() - time > 200 &&
        RobotContainer.operatorInterrupt();
  }
}
