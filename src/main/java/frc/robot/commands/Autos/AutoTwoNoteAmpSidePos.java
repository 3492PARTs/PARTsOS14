// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmToPositionAutoCmd;
import frc.robot.commands.Drive.DriveAngleCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.ZeroDriveMotors;
import frc.robot.commands.IntakeShoot.RunIntakePhotoEyeAutoCmd;
import frc.robot.commands.IntakeShoot.ShootInAmpCmd;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoNoteAmpSidePos extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteRightPos. */
  public AutoTwoNoteAmpSidePos(int red) {
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
            // moves arm to angle that shoots in speaker from the side
            new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
            // shoots in speaker
            new ShootInSpeakerCmd())),
        // drives FORWARD 10 inches
        new DriveDistanceCmd(Units.inchesToMeters(10)).withTimeout(2),
        // turns LEFT face note
        new DriveAngleCmd(-26.5 * red),
        // moves arm to ground
        new ArmToPositionAutoCmd(Constants.Arm.GROUND),

        new ParallelCommandGroup(
            // drives forward while...
            new DriveDistanceCmd(Units.inchesToMeters(60)),
            // ...running intake
            new RunIntakePhotoEyeAutoCmd(Constants.Intake.INTAKE_SPEED)),
        // moves arm to home
        new ArmToPositionAutoCmd(Constants.Arm.HOME),
        // move forward 6.5
        new DriveDistanceCmd(Units.inchesToMeters(6.5)),
        // turn LEFT to amp
        new DriveAngleCmd(-45 * red),
        // drive to amp
        new DriveDistanceCmd(Units.inchesToMeters(-40)),
        // moves arm to amp
        new ArmToPositionAutoCmd(Constants.Arm.AMP),
        // shoot in amp
        new ShootInAmpCmd(),

        new ParallelCommandGroup(
            //moves arm to home...
            new ArmToPositionAutoCmd(Constants.Arm.HOME),
            //while driving forward
            new DriveDistanceCmd(Units.inchesToMeters(5))),

        //turn RIGHT toward center
        new DriveAngleCmd(45 * red),

        new DriveDistanceCmd(Units.inchesToMeters(60)));

  }
}
