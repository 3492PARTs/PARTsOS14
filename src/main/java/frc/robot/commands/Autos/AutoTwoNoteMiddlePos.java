// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.TimerCmd;
import frc.robot.commands.Arm.ArmToPositionAutoCmd;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.RunArmToZeroCmd;
import frc.robot.commands.Arm.ZeroArmCmd;
import frc.robot.commands.Arm.ZeroPivotEncodersCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Drive.ZeroDriveEncodersCmd;
import frc.robot.commands.Drive.ZeroDriveMotors;
import frc.robot.commands.IntakeShoot.RunIntakePhotoEyeCmd;
import frc.robot.commands.IntakeShoot.BangBangShooterCmd;
import frc.robot.commands.IntakeShoot.RunIntakeAtRPMCmd;
import frc.robot.commands.IntakeShoot.TimeIntakeCmd;

public class AutoTwoNoteMiddlePos extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteMiddlePos. */
  public AutoTwoNoteMiddlePos() {
    addCommands(new ParallelRaceGroup(new ZeroDriveMotors(),
        new SequentialCommandGroup(
            new ZeroArmCmd(),
            // Move arm to angle and warm up shooter
            new ParallelRaceGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
                new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
            // Shoot once at speed
            new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
                new RunIntakeAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
                new HoldArmInPositionCmd(Constants.Arm.SPEAKER)),
            // Move arm to ground
            new ProfiledPivotArmCmd(Constants.Arm.GROUND))),
        // Turn on intake and move forward
        new DriveDistanceCmd(Units.inchesToMeters(36)),
        new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED),
        // Pivot up and speed
        new ParallelCommandGroup(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
            new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)),
        // Shoot
        new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeAtRPMCmd(Constants.Shooter.SPEAKER_RPM),
            new HoldArmInPositionCmd(Constants.Arm.SPEAKER)));
  }
}
