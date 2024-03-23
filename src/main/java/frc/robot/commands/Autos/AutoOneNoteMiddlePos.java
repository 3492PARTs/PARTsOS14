// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.IntakeShoot.RunIntakeAtRPMCmd;

public class AutoOneNoteMiddlePos extends SequentialCommandGroup {
  /** Creates a new AutoOneNoteMiddlePos. */
  public AutoOneNoteMiddlePos() {
    // Set arm pos then shoot in speaker.
    addCommands(new ArmToPositionTeleopCmd(Constants.Arm.SPEAKER),
        new RunIntakeAtRPMCmd(Constants.Shooter.SPEAKER_RPM));
  }
}
