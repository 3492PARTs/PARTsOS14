// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Intake.TimeIntakeCmd;
import frc.robot.commands.Shooter.TimeShootCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestSubsystemsCmdSeq extends SequentialCommandGroup {
  /** Creates a new TestSubsystemsCmdSeq. */
  public TestSubsystemsCmdSeq() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    WaitCommand wait = new WaitCommand(0.5);
    addCommands(new ZeroArmCmdSeq(), wait, new PIDDriveCmd(1), wait,
        new PIDDriveCmd(-1), wait, new ProfiledPivotArmCmd(Constants.Arm.HOME), wait, new TimeIntakeCmd(0.5, 1), wait,
        new TimeIntakeCmd(0.5, -1),
        wait, new TimeShootCmd(0.5, 0.5), wait, new TimeShootCmd(0.5, -0.5), wait,
        new ProfiledPivotArmCmd(Constants.Arm.GROUND));
  }
}
