// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Candle.Color;

public class RunIntakeWhenShooterAtRPMCmd extends Command {
  /** Creates a new ShootInAmpCmd. */
  Shooter shooter;
  Intake intake;
  Candle candle;
  double RPM;
  long startTime;
  boolean setCandleBlink = true;

  public RunIntakeWhenShooterAtRPMCmd(double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    candle = Candle.getInstance();
    this.RPM = RPM;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intake.hasNote())
      this.startTime = System.currentTimeMillis();

    if (setCandleBlink && shooter.getShooterRPM() >= 0.8 * RPM) {
      candle.runFadeAnimation(Color.YELLOW);
      setCandleBlink = false;
    }

    if (shooter.getShooterRPM() >= RPM) {
      intake.runIntake(-.9);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // 200 ms after note is gone stop command
    return System.currentTimeMillis() - this.startTime > 200;
  }
}
