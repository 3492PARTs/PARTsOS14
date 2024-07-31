// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoldArmInPositionCmd extends ProfiledPIDCommand {
  /** Creates a new HoldArmCmd. */
  double angleSetpoint;

  public HoldArmInPositionCmd(double angleSetpoint) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            8,
            0,
            0,
           Arm.getInstance().getConstraints()),
        // This should return the measurement
        () -> Arm.getInstance().getCurrentState().position, // getCurrentState() is a trapezoid profile object
        // This should return the goal (can also be a constant)
        new TrapezoidProfile.State(Math.toRadians(angleSetpoint), 0).position,
        // This uses the output
        (output, setpoint) -> {

          double volts = Arm.getInstance().calcOutputVoltage(setpoint.velocity);
          double finalOutput = -(volts + output);


          if (Arm.getInstance().getLimitSwitch() && finalOutput > 0)
            Arm.getInstance().setVolts(0);
          else
            Arm.getInstance().setVolts(finalOutput);
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1);
    this.angleSetpoint = angleSetpoint;
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    Arm.getInstance().setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return RobotContainer.operatorJoystickInterrupt() ||
        RobotContainer.operatorButtonInterrupt();
  }
}
