// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Intake.RunIntakePhotoEyeCmd;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeArmToPositionCmdSeq extends SequentialCommandGroup {
  /** Creates a new IntakeArmPosCmdSeq. */
  public IntakeArmToPositionCmdSeq(double speed, double armPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunIntakePhotoEyeCmd(speed),
        new ConditionalCommand(
            new ParallelCommandGroup(new OnIntakeOuttakeCmdSeq(), new ProfiledPivotArmCmd(armPosition)),
            new IntakeCmdSeq(speed),
            Intake.getInstance().hasNoteSupplier()));

    //new ParallelCommandGroup(new TimeIntakeCmd(.2, .38), new ProfiledPivotArmCmd(armPosition))
    //  .onlyIf(Intake.getInstance().hasNoteSupplier()));
  }
}
