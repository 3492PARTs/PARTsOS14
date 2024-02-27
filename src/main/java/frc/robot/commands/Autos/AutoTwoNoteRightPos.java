// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
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
public class AutoTwoNoteRightPos extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteRightPos. */
  public AutoTwoNoteRightPos() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(new ZeroDriveMotors(),
        new SequentialCommandGroup(
            // moves arm to angle that shoots in speaker from the side
            new ArmToPositionAutoCmd(Constants.Arm.SPEAKER_SIDE_ANGLE),
            // shoots in speaker
            new ShootInSpeakerCmd())),
        // drives FORWARD 10 inches
        new DriveDistanceCmd(Units.inchesToMeters(10)).withTimeout(2),
        // turns LEFT face note
        new DriveAngleCmd(-26.5),
        // moves arm to ground
        new ArmToPositionAutoCmd(76),
        new ParallelCommandGroup(
            // drives forward while...
            new DriveDistanceCmd(Units.inchesToMeters(60)),
            // ...running intake
            new RunIntakePhotoEyeAutoCmd(Constants.Intake.INTAKE_SPEED)),
        // moves arm to home
        new ArmToPositionAutoCmd(Constants.Arm.HOME),
        // move forward 6.5
        new DriveDistanceCmd(Units.inchesToMeters(6.5)),
        // turn to amp
        new DriveAngleCmd(-45),
        // drive to amp
        new DriveDistanceCmd(Units.inchesToMeters(-40)),
        // moves arm to amp
        new ArmToPositionAutoCmd(Constants.Arm.AMP),
        // shoot in amp
        new ShootInAmpCmd(),
        new ArmToPositionAutoCmd(Constants.Arm.HOME),
        new DriveDistanceCmd(Units.inchesToMeters(5)),
        new DriveAngleCmd(45),
        new DriveDistanceCmd(Units.inchesToMeters(40)));
  }
}
