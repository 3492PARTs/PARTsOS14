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
        new SequentialCommandGroup(new ArmToPositionAutoCmd(Constants.Arm.SPEAKER_SIDE_ANGLE),
            new ShootInSpeakerCmd())),
        new DriveDistanceCmd(Units.inchesToMeters(10)).withTimeout(2),
        new DriveAngleCmd(-26.5),
        new ArmToPositionAutoCmd(76),
        new ParallelCommandGroup(new DriveDistanceCmd(Units.inchesToMeters(60)),
            new RunIntakePhotoEyeAutoCmd(Constants.Intake.INTAKE_SPEED)),
        new ArmToPositionAutoCmd(Constants.Arm.HOME),
        new DriveDistanceCmd(Units.inchesToMeters(6.5)),
        new DriveAngleCmd(-45),
        new DriveDistanceCmd(Units.inchesToMeters(-40)),
        new ArmToPositionAutoCmd(Constants.Arm.AMP),
        new ShootInAmpCmd(),
        new ArmToPositionAutoCmd(Constants.Arm.HOME),
        new DriveDistanceCmd(Units.inchesToMeters(5)),
        new DriveAngleCmd(45),
        new DriveDistanceCmd(Units.inchesToMeters(40)));
  }
}
