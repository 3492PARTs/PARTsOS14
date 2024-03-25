// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.EmptySide;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionAutoCmd;
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.StopDriveMotorsCmd;
import frc.robot.commands.Intake.RunIntakePhotoEyeCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.TimeIntakeCmd;

public class AutoTwoNoteEmptySide extends SequentialCommandGroup {
    /** Creates a new AutoTwoNoteLeftPos. */
    public AutoTwoNoteEmptySide(int red) {
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

        addCommands(new ParallelRaceGroup(new StopDriveMotorsCmd(),
                new SequentialCommandGroup(
                        new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
                        new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM))),
                new DriveDistanceCmd(Units.inchesToMeters(8)).withTimeout(2),
                // TODO: increase angle
                new DriveAngleCmd(27.5 * red),
                new ArmToPositionAutoCmd(76),
                new ParallelCommandGroup(
                        // drives forward while...
                        // TODO: drive more forward
                        new DriveDistanceCmd(Units.inchesToMeters(60)),
                        // ...running intake
                        new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED)),
                new TimeIntakeCmd(.2, .2),
                new ParallelCommandGroup(
                        new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
                        new DriveDistanceCmd(Units.inchesToMeters(-60))),
                new DriveAngleCmd(-26.5 * red),
                new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM));
    }
}
