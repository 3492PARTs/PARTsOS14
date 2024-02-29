// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionAutoCmd;
import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.ZeroDriveMotors;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoOneNoteEmptySide extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteLeftPos. */
  public AutoOneNoteEmptySide(int red) {
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
    addCommands(new ParallelRaceGroup(new ZeroDriveMotors(),

        new SequentialCommandGroup(
            new ArmToPositionAutoCmd(Constants.Arm.SPEAKER_SIDE_ANGLE),
            new ShootInSpeakerCmd())),

        // TODO: fix driving distance to be longer
        new DriveDistanceCmd(Units.inchesToMeters(145)).withTimeout(2),
        new DriveAngleCmd(26.5 * red),
        // TODO: fix driving distance to be longer
        new DriveDistanceCmd(Units.inchesToMeters(100)));
  }
}
