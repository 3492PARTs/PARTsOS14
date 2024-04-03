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
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.Sequences.IntakeCmdSeq;
import frc.robot.commands.Shooter.BangBangShooterCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoThreeNoteMiddle extends SequentialCommandGroup {
        /** Creates a new AutoFourNoteMiddlePosition. */
        public AutoThreeNoteMiddle(int red, boolean ampOrEmpty, boolean speakerOrAmp) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(new ZeroArmCmdSeq(),
                                // Move arm to angle and warm up shooter
                                new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                                                new BangBangShooterCmd(
                                                                Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                // Shoot preload note 1
                                new ParallelRaceGroup(
                                                new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                //Move arm to ground
                                new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                //  turn on intake, and move forward
                                new ParallelCommandGroup(
                                                new IntakeCmdSeq(Constants.Intake.INTAKE_SPEED), //.withTimeout(5),
                                                new DriveDistanceCmd(Units.inchesToMeters(36))),
                                // Pivot up and speed
                                new ParallelRaceGroup(
                                                new ParallelCommandGroup(
                                                                new DriveDistanceCmd(Units.inchesToMeters(-36)),
                                                                new ProfiledPivotArmCmd(Constants.Arm.SPEAKER)),
                                                new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
                                // Shoot note 2
                                new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                                                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                                                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
                                // drive forward to line up with note
                                new DriveDistanceCmd(Units.inchesToMeters(40.0)),
                                // Go amp or Go Empty side
                                new ConditionalCommand(
                                                // Condition: Turn to empty side note
                                                new SequentialCommandGroup(
                                                                new ParallelCommandGroup(
                                                                                //turn to note
                                                                                new DriveAngleCmd(-90 * red), //-75
                                                                                //Move arm to ground
                                                                                new ProfiledPivotArmCmd(
                                                                                                Constants.Arm.GROUND)),
                                                                //  turn on intake, and move forward
                                                                new ParallelCommandGroup(
                                                                                new IntakeCmdSeq(
                                                                                                Constants.Intake.INTAKE_SPEED), //.withTimeout(5),
                                                                                new DriveDistanceCmd(Units
                                                                                                .inchesToMeters(36))),
                                                                // Back up to avoid stage
                                                                new DriveDistanceCmd(-Units.inchesToMeters(34)),
                                                                // Turn, raise arm, and speed up
                                                                new ParallelRaceGroup(new BangBangShooterCmd(
                                                                                Constants.Shooter.WARMUP_SPEAKER_RPM),
                                                                                new ParallelCommandGroup(
                                                                                                new ProfiledPivotArmCmd(
                                                                                                                Constants.Arm.SPEAKER_BACK_30),
                                                                                                new DriveAngleCmd(90
                                                                                                                * red))),
                                                                // Shoot note 3
                                                                new ParallelRaceGroup(new BangBangShooterCmd(
                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                new RunIntakeWhenShooterAtRPMCmd(
                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                new HoldArmInPositionCmd(
                                                                                                Constants.Arm.SPEAKER_BACK_30))

                                                ),

                                                // Condition: Turn to amp side note
                                                new SequentialCommandGroup(
                                                                new ParallelCommandGroup(
                                                                                // Turn to note
                                                                                new DriveAngleCmd(80 * red),
                                                                                //Move arm to ground
                                                                                new ProfiledPivotArmCmd(
                                                                                                Constants.Arm.GROUND)),
                                                                //  turn on intake, and move forward
                                                                new ParallelCommandGroup(
                                                                                new IntakeCmdSeq(
                                                                                                Constants.Intake.INTAKE_SPEED), //.withTimeout(5),
                                                                                new DriveDistanceCmd(
                                                                                                Units.inchesToMeters(
                                                                                                                36))),
                                                                // Shoot speaker or shoot amp
                                                                new ConditionalCommand(
                                                                                // Condition: robot goes to speaker 
                                                                                new SequentialCommandGroup(
                                                                                                // turn, raise arm, and speed up
                                                                                                new ParallelRaceGroup(
                                                                                                                new BangBangShooterCmd(
                                                                                                                                Constants.Shooter.WARMUP_SPEAKER_RPM),
                                                                                                                new ParallelCommandGroup(// turn to speaker
                                                                                                                                new DriveAngleCmd(
                                                                                                                                                -70
                                                                                                                                                                * red),
                                                                                                                                new ProfiledPivotArmCmd(
                                                                                                                                                38))),
                                                                                                // Shoot
                                                                                                new ParallelRaceGroup(
                                                                                                                new BangBangShooterCmd(
                                                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                                                new RunIntakeWhenShooterAtRPMCmd(
                                                                                                                                Constants.Shooter.SPEAKER_RPM),
                                                                                                                new HoldArmInPositionCmd(
                                                                                                                                38)),
                                                                                                new DriveAngleCmd(-10
                                                                                                                * red)),
                                                                                //robot goes to amp
                                                                                new SequentialCommandGroup(
                                                                                                // Turn to amp
                                                                                                new DriveAngleCmd(180),
                                                                                                new ParallelCommandGroup( // Back up to amp
                                                                                                                new DriveDistanceCmd(
                                                                                                                                Units.inchesToMeters(
                                                                                                                                                -10)),
                                                                                                                // Move arm to amp
                                                                                                                new ProfiledPivotArmCmd(
                                                                                                                                Constants.Arm.AMP)),

                                                                                                // Shoot
                                                                                                new ParallelRaceGroup(
                                                                                                                new BangBangShooterCmd(
                                                                                                                                Constants.Shooter.AMP_RPM),
                                                                                                                new RunIntakeWhenShooterAtRPMCmd(
                                                                                                                                Constants.Shooter.AMP_RPM),
                                                                                                                new HoldArmInPositionCmd(
                                                                                                                                Constants.Arm.AMP)),
                                                                                                new ParallelCommandGroup(
                                                                                                                // Home arm 
                                                                                                                new ProfiledPivotArmCmd(
                                                                                                                                Constants.Arm.HOME),
                                                                                                                // Drive away from amp
                                                                                                                new DriveDistanceCmd(
                                                                                                                                Units.inchesToMeters(
                                                                                                                                                10))),
                                                                                                // Turn to center 
                                                                                                new DriveAngleCmd(90
                                                                                                                * red)),

                                                                                () -> {
                                                                                        return speakerOrAmp;
                                                                                })),
                                                () -> {
                                                        return ampOrEmpty;
                                                }),
                                // Home arm and Drive to center

                                new ParallelRaceGroup(new DriveDistanceCmd(Units.inchesToMeters(72)),
                                                new HoldArmInPositionCmd(Constants.Arm.HOME)),
                                // Hold arm in position till finish
                                new HoldArmInPositionCmd(Constants.Arm.HOME));

        }
}
