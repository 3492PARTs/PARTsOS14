// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Logger;
import frc.robot.util.StopWatch;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDShootCmd extends PIDCommand {
  StopWatch stopWatch = new StopWatch();

  /** Creates a new PIDShootCmd. */
  public PIDShootCmd(double setpointRPM) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> Shooter.getInstance().getShooterRPM(),
        // This should return the setpoint (can also be a constant)
        () -> setpointRPM,
        // This uses the output
        output -> {
          // Use the output here
          Shooter.getInstance().setSpeed(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    Logger.getInstance().logString(this.getName(), "start");
    stopWatch.start();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Logger.getInstance().logString(this.getName(), String.format("end, interrupted: %s"));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // stopwatch stops interrupt from happening immediately.
    return stopWatch.getMilliseconds() > 200 && RobotContainer.operatorInterrupt();
  }
}
