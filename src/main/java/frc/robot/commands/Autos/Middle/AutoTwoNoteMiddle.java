// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Middle;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Intake.RunIntakePhotoEyeCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.subsystems.Intake;

public class AutoTwoNoteMiddle extends SequentialCommandGroup {
        /** Creates a new AutoTwoNoteMiddlePos. */
        public AutoTwoNoteMiddle() {
                addCommands(new ZeroArmCmdSeq(),
                                // Move arm to angle and warm up shooter
                                new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                new BangBangShooterCmd(
                                                                Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                // Shoot
                                new ParallelRaceGroup(
                                                new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                // Move arm to ground, turn on intake, and move forward
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                                new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED).withTimeout(5),
                                                new PIDDriveCmd(Units.inchesToMeters(36))),
                                // Conditional group, for getting the note or not
                                new ConditionalCommand(
                                                // has note
                                                new SequentialCommandGroup(
                                                                // Pivot up and speed
                                                                new ParallelCommandGroup(
                                                                                new ProfiledPivotArmCmd(
                                                                                                Constants.Arm.SPEAKER),
                                                                                new BangBangShooterCmd(
                                                                                                Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                                                // Shoot 
                                                                new ParallelRaceGroup(
                                                                                new BangBangShooterCmd(
                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                new RunIntakeWhenShooterAtRPMCmd(
                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                new HoldArmInPositionCmd(
                                                                                                Constants.Arm.SPEAKER))),
                                                // does not have note, pivot arm to home
                                                new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                () -> {
                                                        return Intake.getInstance().hasNote();
                                                }),
                                // drive to center 
                                new PIDDriveCmd(Units.inchesToMeters(36))

                );
        }
}
