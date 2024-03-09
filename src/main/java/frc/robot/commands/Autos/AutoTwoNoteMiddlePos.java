// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.TimerCmd;
import frc.robot.commands.Arm.ArmToPositionAutoCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.ZeroDriveMotors;
import frc.robot.commands.IntakeShoot.RunIntakePhotoEyeAutoCmd;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;
import frc.robot.commands.IntakeShoot.TimeIntakeCmd;

public class AutoTwoNoteMiddlePos extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteMiddlePos. */
  public AutoTwoNoteMiddlePos() {
    addCommands(
        new ParallelRaceGroup(new ZeroDriveMotors(),
            new SequentialCommandGroup(
                // Moves arm to speaker.
                new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
                // Shoots in speaker.
                new ShootInSpeakerCmd(),
                // Moves arm to ground.
                new ArmToPositionAutoCmd(Constants.Arm.GROUND + 2))),
        new TimerCmd(.5),
        new ParallelCommandGroup(
            // Drives forward while...
            new DriveDistanceCmd(Units.inchesToMeters(36)),
            // ...running intake.
            new RunIntakePhotoEyeAutoCmd(Constants.Intake.INTAKE_SPEED + .2)),
        new TimeIntakeCmd(.2, .2),
        // Moves arm up to speaker position after has note.
        new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
        // Drives backward 4 inches.
        new DriveDistanceCmd(Units.inchesToMeters(-36)),
        // Shoots in speaker.
        new ShootInSpeakerCmd(),
        //Moves forward as far into the field.
        new DriveDistanceCmd(Units.inchesToMeters(50)));
  }
}
