// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeShoot.IntakePhotoEyeArmPosCmd;
import frc.robot.subsystems.Arm;

public class ProfiledPivotArmCmd extends ProfiledPIDCommand {

  double angle;

  /** Creates a new profiledPivotArm. */
  public ProfiledPivotArmCmd(double angle) {
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
        new TrapezoidProfile.State(Math.toRadians(angle), 0).position,
        // This uses the output
        (output, setpoint) -> {

          double volts = Arm.getInstance().calcOutputVoltage(setpoint.velocity);
          Arm.getInstance().driveMotorVolts(-(volts + output));
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1);
    this.angle = angle;
  }

  @Override
  public void end(boolean interrupted) {
    Arm.getInstance().setPivotSpeed(0);
    // Once arm is on the ground, run the intake to pick up a note.
    if (angle == Constants.Arm.GROUND) {
      new IntakePhotoEyeArmPosCmd(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
