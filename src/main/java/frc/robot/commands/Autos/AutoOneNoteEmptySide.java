// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionAutoCmd;
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.ZeroDriveMotors;
import frc.robot.commands.IntakeShoot.RunIntakeAtRPMCmd;

public class AutoOneNoteEmptySide extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteLeftPos. */
  //? Empty side??
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
            new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
            new RunIntakeAtRPMCmd(Constants.Shooter.SPEAKER_RPM))),

        // TODO: fix driving distance to be longer
        new DriveDistanceCmd(Units.inchesToMeters(163)).withTimeout(2),
        new DriveAngleCmd(26.5 * red),
        // TODO: fix driving distance to be longer
        new DriveDistanceCmd(Units.inchesToMeters(148)));
  }
}
