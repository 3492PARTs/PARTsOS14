// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Drive.PIDTurnCmd;
import frc.robot.commands.Intake.RunIntakePhotoEyeCmd;
import frc.robot.commands.Intake.RunIntakeWhenAtRPMCmd;
import frc.robot.commands.Shooter.BangBangShooterCmd;

public class AutoTwoNoteMiddlePos extends SequentialCommandGroup {
        /** Creates a new AutoTwoNoteMiddlePos. */
        public AutoTwoNoteMiddlePos() {
                int red = 1;
                addCommands(new ZeroArmCmdSeq(),
                                // Move arm to angle and warm up shooter
                                new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                new BangBangShooterCmd(
                                                                Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                // Shoot
                                new ParallelRaceGroup(
                                                new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                // Move arm to ground, urn on intake, and move forward
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                                new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED),
                                                new PIDDriveCmd(Units.inchesToMeters(36))),
                                // Pivot up and speed
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                // Shoot
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                //TODO Untested below
                                // Turn to empty side note
                                new PIDTurnCmd(-90 * red),
                                // Move arm to ground, urn on intake, and move forward
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                                new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED),
                                                new PIDDriveCmd(Units.inchesToMeters(36))),
                                // Back up to avoid stage
                                new PIDDriveCmd(-Units.inchesToMeters(36)),
                                // Turn raise arm and speed up
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM),
                                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                                new PIDTurnCmd(90 * red))),
                                // Shoot
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),

                                // Turn to amp side note
                                new PIDTurnCmd(90 * red),
                                // Move arm to ground, urn on intake, and move forward
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                                new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED),
                                                new PIDDriveCmd(Units.inchesToMeters(36))),
                                // Turn raise arm and speed up
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM),
                                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                                new PIDTurnCmd(-45 * red))),
                                // Shoot
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                // Turn to center and home arm
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                new PIDTurnCmd(45 * red)),
                                // Drive to center
                                new PIDDriveCmd(Units.inchesToMeters(36)));
        }
}
