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
import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.ZeroDriveMotors;
import frc.robot.commands.IntakeShoot.RunIntakeCmd;
import frc.robot.commands.IntakeShoot.RunIntakePhotoEyeAutoCmd;
import frc.robot.commands.IntakeShoot.RunIntakePhotoEyeTeleopCmd;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoNoteMiddlePos extends SequentialCommandGroup {
  /** Creates a new AutoTwoNoteMiddlePos. */
  public AutoTwoNoteMiddlePos() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(new ZeroDriveMotors(),
            new SequentialCommandGroup(
                // moves arm to speaker
                new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
                // shoots in speaker
                new ShootInSpeakerCmd(),
                // moves arm to ground
                new ArmToPositionAutoCmd(Constants.Arm.GROUND))),
        new ParallelCommandGroup(
            // drives forward while...
            new DriveDistanceCmd(Units.inchesToMeters(35)),
            // ...running intake
            new RunIntakePhotoEyeAutoCmd(Constants.Intake.INTAKE_SPEED)),
        // moves arm up to speaker position after has note
        new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
        // drives backward 4 inches
        new DriveDistanceCmd(Units.inchesToMeters(-35)),
        // shoots in speaker
        new ShootInSpeakerCmd(),
        //moves forward as far into the field
        new DriveDistanceCmd(Units.inchesToMeters(50)));
  }
}
