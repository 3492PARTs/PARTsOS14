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
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.ZeroDriveMotors;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;

public class AutoOneNoteAmpSidePos extends SequentialCommandGroup {

  int red = 1;
  /** Creates a new AutoOneNoteRightPos.
   * @param red ?? - Rewrite later.
  */
  public AutoOneNoteAmpSidePos(int red) {
    
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    // Wrapped if statement to avoid the NoSuchElementException throw.
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        red = 1;
      }
      if (ally.get() == Alliance.Blue) {
        red = -1;
      }
    }
    

    addCommands(new ParallelRaceGroup(new ZeroDriveMotors(),
        new SequentialCommandGroup(
            // moves arm to angle that shoots in speaker from the side
            new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
            // shoots in speaker
            new ShootInSpeakerCmd())),
        // drives FORWARD 10 inches
        new DriveDistanceCmd(Units.inchesToMeters(10)).withTimeout(2),
        // turns LEFT face note
        new DriveAngleCmd(-26.5 * red),
        new DriveDistanceCmd(Units.inchesToMeters(60)));
  }
}
