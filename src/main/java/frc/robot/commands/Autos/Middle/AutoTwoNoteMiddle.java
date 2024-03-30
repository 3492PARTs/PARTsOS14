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
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Intake.RunIntakePhotoEyeCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.Sequences.IntakeCmdSeq;
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
                                //Move arm to ground
                                new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                //  turn on intake, and move forward
                                new ParallelCommandGroup(
                                                new IntakeCmdSeq(Constants.Intake.INTAKE_SPEED).withTimeout(5),
                                                new DriveDistanceCmd(Units.inchesToMeters(36))),
                                // Conditional group, for getting the note or not
                                new ConditionalCommand(
                                                // has note
                                                new SequentialCommandGroup(
                                                                // Drive back to speaker
                                                                new DriveDistanceCmd(-Units.inchesToMeters(36)),
                                                                // Pivot up and speed
                                                                new ParallelRaceGroup(
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
                                                                                                Constants.Arm.SPEAKER)),
                                                                // Drive back to where note was
                                                                new DriveDistanceCmd(-Units.inchesToMeters(36))),
                                                // does not have note, pivot arm to home
                                                new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                Intake.getInstance().hasNoteSupplier()),
                                // drive to center 
                                new DriveDistanceCmd(Units.inchesToMeters(36))

                );
        }
}
