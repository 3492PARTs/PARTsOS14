// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Middle;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.PivotArmShooterSpeedCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.PIDTurnCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.Sequences.IntakeCmdSeq;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.commands.Shooter.Sequences.ShootAtRPMCmdSeq;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFourNoteMiddleCopy extends SequentialCommandGroup {
        /** Creates a new AutoFourNoteMiddlePosition. */
        public AutoFourNoteMiddleCopy(int red) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(new ZeroArmCmdSeq(),
                                // Move arm to angle and warm up shooter
                                new PivotArmShooterSpeedCmdSeq(Constants.Arm.SPEAKER,
                                                Constants.Shooter.WARMUP_SPEAKER_RPM),
                                // Shoot preload note 1
                                new ShootAtRPMCmdSeq(Constants.Shooter.SPEAKER_RPM, Constants.Arm.SPEAKER),
                                //Move arm to ground
                                new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                //  turn on intake, and move forward
                                new ParallelCommandGroup(
                                                new IntakeCmdSeq(Constants.Intake.INTAKE_SPEED), //.withTimeout(5),
                                                new DriveDistanceCmd(Units.inchesToMeters(36))),
                                // Pivot up and speed
                                new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER_BACK_30),
                                                new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                // Shoot note 2
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER_BACK_30)),
                                // drive forward to line up with next note
                                new DriveDistanceCmd(Units.inchesToMeters(1.1)),
                                new ParallelCommandGroup(
                                                // Turn to empty side note
                                                new PIDTurnCmd(-90 * red), //-75
                                                //Move arm to ground
                                                new ProfiledPivotArmCmd(Constants.Arm.GROUND)),
                                //  turn on intake, and move forward
                                new ParallelCommandGroup(
                                                new IntakeCmdSeq(Constants.Intake.INTAKE_SPEED), //.withTimeout(5),
                                                new DriveDistanceCmd(Units.inchesToMeters(36))),

                                // Back up to avoid stage
                                new DriveDistanceCmd(-Units.inchesToMeters(34)),
                                // Turn, raise arm, and speed up
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM),
                                                new ParallelCommandGroup(
                                                                new ProfiledPivotArmCmd(Constants.Arm.SPEAKER_BACK_30),
                                                                new PIDTurnCmd(90 * red))),
                                // Shoot note 3
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER_BACK_30)),
                                new ParallelCommandGroup(
                                                // Turn to amp side note
                                                new PIDTurnCmd(90 * red),
                                                //Move arm to ground
                                                new ProfiledPivotArmCmd(Constants.Arm.GROUND)),
                                //  turn on intake, and move forward
                                new ParallelCommandGroup(
                                                new IntakeCmdSeq(Constants.Intake.INTAKE_SPEED), //.withTimeout(5),
                                                new DriveDistanceCmd(Units.inchesToMeters(36))),
                                // Turn, raise arm, and speed up
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM),
                                                new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                new PIDTurnCmd(-70 * red)),
                                // Shoot note 4
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                // Turn to center and home arm
                                new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                new PIDTurnCmd(-30 * red)),
                                new ParallelRaceGroup(// Drive to center
                                                new DriveDistanceCmd(Units.inchesToMeters(156)),
                                                new HoldArmInPositionCmd(Constants.Arm.HOME)),
                                // hold arm until end of auto
                                new HoldArmInPositionCmd(Constants.Arm.HOME));

        }
}
