// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AmpSide;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionAutoCmd;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Drive.PIDTurnCmd;
import frc.robot.commands.Drive.StopDriveMotorsCmd;
import frc.robot.commands.Intake.RunIntakePhotoEyeCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.TimeIntakeCmd;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.commands.Shooter.ShootInAmpCmd;
import frc.robot.subsystems.Intake;

public class AutoTwoNoteAmpSide extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteRightPos. */
  public AutoTwoNoteAmpSide(int red) {
    /*
    int red = 1;
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        red = 1;
      }
      if (ally.get() == Alliance.Blue) {
        red = -1;
      }
    }
    */

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
        new PIDTurnCmd(-41.5 * red),
                    // Move arm to ground, turn on intake, and move forward
                    new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.GROUND),
                                    new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED).withTimeout(5),
                                    new PIDDriveCmd(Units.inchesToMeters(60))),
                    // Conditional group, for getting the note or not
                    new ConditionalCommand(
                                    // has note
                                    new SequentialCommandGroup(
                                                    //Move arm to home, drive forward
                                                    new ParallelCommandGroup(
                                                                  new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                                  new PIDDriveCmd(Units.inchesToMeters(6.5))
                                                    ),
                                                    //turn left to amp
                                                    new PIDTurnCmd(-45 * red),
                                                    //move are to amp and drive backward
                                                    new ParallelCommandGroup(
                                                                  new ProfiledPivotArmCmd(Constants.Arm.AMP),
                                                                  new PIDDriveCmd(-40)
                                                    ),
                                                    // Shoot in amp
                                                    new ParallelRaceGroup(
                                                                  new BangBangShooterCmd(Constants.Shooter.AMP_RPM),
                                                                  new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.AMP_RPM),
                                                                  new HoldArmInPositionCmd(Constants.Arm.AMP)),
                                                    //move arm to home and drive forward
                                                    new ParallelCommandGroup(
                                                                  new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                                  new PIDDriveCmd(Units.inchesToMeters(5))
                                                    ),
                                                    //turn to center
                                                    new PIDTurnCmd(45 * red)),
                                                // does not have note, pivot arm to home
                                                new ProfiledPivotArmCmd(Constants.Arm.HOME),
                                                () -> {
                                                        return Intake.getInstance().hasNote();
                                                }),
                    //drive to center
                    new PIDDriveCmd(Units.inchesToMeters(60)));                            
  }
}
