// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AmpSide;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Drive.PIDTurnCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Shooter.BangBangShooterCmd;

public class AutoOneNoteAmpSide extends SequentialCommandGroup {
  public AutoOneNoteAmpSide(int red) {

    addCommands(new ZeroArmCmdSeq(),
        // Move arm to angle and warm up shooter
        new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
            new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
        // Shoot
        new ParallelRaceGroup(
            new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
            new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
        // drives FORWARD 10 inches
        new PIDDriveCmd(Units.inchesToMeters(10)).withTimeout(2),
        // turns LEFT face note
        new PIDTurnCmd(-26.5 * red),
        //drive forward to next note
        new PIDDriveCmd(Units.inchesToMeters(60)));

  }
}
