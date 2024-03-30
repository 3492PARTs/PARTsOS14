// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.util.Logger;

public class ProfiledPivotArmCmd extends ProfiledPIDCommand {

  double angleSetpoint;

  /** Creates a new profiledPivotArm. */
  public ProfiledPivotArmCmd(double angleSetpoint) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains (tune later)
            Constants.Arm.kP, // 2,7
            Constants.Arm.kI,
            Constants.Arm.kD,
            // The motion profile constraints
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
    System.out.println("angle setpoint " + angleSetpoint);
  }

  @Override
  public void execute() {
    super.execute();
    if (angleSetpoint == Constants.Arm.GROUND && getController().atGoal() && !Arm.getInstance().getLimitSwitch()) {
      Arm.getInstance().setSpeed(0.1);
      System.out.println("ARM EXTRA BOTTOM");
    }
  }

  @Override
  public void end(boolean interrupted) {
    Arm.getInstance().setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("at goal " + getController().atGoal());
    System.out.println("arm " + Arm.getInstance().getLimitSwitch());
    if (angleSetpoint == Constants.Arm.GROUND) {
      return Arm.getInstance().getLimitSwitch();
    } else {
      return getController().atGoal();
    }

  }
}
