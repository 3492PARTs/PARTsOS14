// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Intake.Sequences.IntakeArmToPositionCmdSeq;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotArmCmdSeq extends SequentialCommandGroup {
  /** Creates a new PivotArmCmdSeq. */
  public PivotArmCmdSeq(double angleSetpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ProfiledPivotArmCmd(angleSetpoint),
        new IntakeArmToPositionCmdSeq(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME).onlyIf(() -> {
          return angleSetpoint == Constants.Arm.GROUND;
        }));
  }
}
