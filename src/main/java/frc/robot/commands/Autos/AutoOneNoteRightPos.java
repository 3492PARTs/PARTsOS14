// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoOneNoteRightPos extends SequentialCommandGroup {
  /** Creates a new AutoOneNoteRightPos. */
  public AutoOneNoteRightPos() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ArmToPositionTeleopCmd(Constants.Arm.SPEAKER_SIDE_ANGLE),
        new ShootInSpeakerCmd());
  }
}