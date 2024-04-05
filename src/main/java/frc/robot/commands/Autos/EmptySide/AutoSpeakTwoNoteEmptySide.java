// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.EmptySide;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.Sequences.IntakeCmdSeq;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.subsystems.Intake;

public class AutoSpeakTwoNoteEmptySide extends SequentialCommandGroup {
        /** Creates a new AutoTwoNoteRightPos. */
        public AutoSpeakTwoNoteEmptySide(int red) {
                addCommands(new ZeroArmCmdSeq(),
                                // Move arm to angle and warm up shooter
                                new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                // Shoot preload note 1
                                new ParallelRaceGroup(
                                                new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                // drives forward to line up with note
                                new DriveDistanceCmd(Units.inchesToMeters(9)),
                                // turns LEFT face note
                                new DriveAngleCmd(47 * red),
                                // Move arm to ground, turn on intake, and move forward
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                                new IntakeCmdSeq(Constants.Intake.INTAKE_SPEED).withTimeout(5),
                                                new DriveDistanceCmd(Units.inchesToMeters(60))),
                                // Conditional group, for getting the note or not
                                new ConditionalCommand(
                                                // has note
                                                new SequentialCommandGroup(
                                                                new ParallelRaceGroup(
                                                                                new BangBangShooterCmd(
                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                new ParallelCommandGroup(
                                                                                                //TODO: fix angle
                                                                                                //Move arm to face speaker
                                                                                                new ProfiledPivotArmCmd(
                                                                                                                Constants.Arm.AMP_NOTE_SPEAKER),
                                                                                                //TODO: fix angle
                                                                                                new DriveAngleCmd(-30
                                                                                                                * red))),
                                                                new ParallelRaceGroup(
                                                                                new BangBangShooterCmd(
                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                new RunIntakeWhenShooterAtRPMCmd(
                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                //TODO: fix angle
                                                                                new HoldArmInPositionCmd(
                                                                                                Constants.Arm.AMP_NOTE_SPEAKER)),
                                                                //TODO: fix angle
                                                                new DriveAngleCmd(20 * red)),

                                                // does not have note, pivot arm to home
                                                new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                Intake.getInstance().hasNoteSupplier()),
                                new ParallelRaceGroup(
                                                //drive to center
                                                new DriveDistanceCmd(Units.inchesToMeters(60)),
                                                new HoldArmInPositionCmd(Constants.Arm.HOME)),
                                // hold arm till end
                                new HoldArmInPositionCmd(Constants.Arm.HOME));
        }
}
