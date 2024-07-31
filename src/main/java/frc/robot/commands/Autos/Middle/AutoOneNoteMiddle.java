// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Middle;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Shooter.BangBangShooterCmd;

public class AutoOneNoteMiddle extends SequentialCommandGroup {
  /** Creates a new AutoOneNoteMiddlePos. */
  public AutoOneNoteMiddle() {
    // Set arm pos then shoot in speaker.
    addCommands(
        new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
            new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
        new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
            new HoldArmInPositionCmd(Constants.Arm.SPEAKER)));
  }
}
