// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.RunArmToLimitSwitchCmd;
import frc.robot.commands.Arm.ZeroPivotEncodersCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.ZeroDriveEncodersCmd;
import frc.robot.commands.IntakeShoot.BangBangShooterCmd;
import frc.robot.commands.IntakeShoot.RunIntakeAtRPMCmd;

public class AutoOneNoteMiddlePos extends SequentialCommandGroup {
  /** Creates a new AutoOneNoteMiddlePos. */
  public AutoOneNoteMiddlePos() {
    // Set arm pos then shoot in speaker.
    addCommands(new ZeroArmCmdSeq(),
        new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
            new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
        new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
            new HoldArmInPositionCmd(Constants.Arm.SPEAKER)));
  }
}
