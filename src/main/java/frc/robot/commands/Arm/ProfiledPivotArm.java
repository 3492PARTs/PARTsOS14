// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PIDValues;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ProfiledPivotArm extends ProfiledPIDCommand {
  
  /** Creates a new profiledPivotArm. */
  public ProfiledPivotArm(double angle, double kP, double kI, double kD) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains (tune later)
            kP,//2,7
            kI,
            kD,
            // The motion profile constraints
            Arm.getInstance().getConstraints()),
        // This should return the measurement
        () -> Arm.getInstance().getCurrentState().position, //getCurrentState() is a trapezoid profile object
        // This should return the goal (can also be a constant)
        new TrapezoidProfile.State(Math.toRadians(angle), 0).position,
        // This uses the output
        (output, setpoint) -> {

          double volts = Arm.getInstance().calcOutputVoltage(setpoint.velocity);
          Arm.getInstance().driveMotorVolts(volts + output);
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return getController().atGoal();
  }
}
