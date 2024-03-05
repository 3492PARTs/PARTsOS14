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
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;
import frc.robot.commands.IntakeShoot.TimeIntakeCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoNoteEmptySpacePos extends SequentialCommandGroup {
    /** Creates a new AutoTwoNoteLeftPos. */
    public AutoTwoNoteEmptySpacePos(int red) {
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
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(new ParallelRaceGroup(new ZeroDriveMotors(),
                new SequentialCommandGroup(
                        new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
                        new ShootInSpeakerCmd())),
                new DriveDistanceCmd(Units.inchesToMeters(8)).withTimeout(2),
                // TODO: increase angle
                new DriveAngleCmd(27.5 * red),
                new ArmToPositionAutoCmd(76),
                new ParallelCommandGroup(
                        // drives forward while...
                        // TODO: drive more forward
                        new DriveDistanceCmd(Units.inchesToMeters(60)),
                        // ...running intake
                        new RunIntakePhotoEyeAutoCmd(Constants.Intake.INTAKE_SPEED)),
                new TimeIntakeCmd(.2, .2),
                new ParallelCommandGroup(
                        new ArmToPositionAutoCmd(Constants.Arm.SPEAKER),
                        new DriveDistanceCmd(Units.inchesToMeters(-60))),
                new DriveAngleCmd(-26.5 * red),
                new ShootInSpeakerCmd());
    }
}
