// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Shooter.BangBangShooterCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAtRPMCmdSeq extends SequentialCommandGroup {
  /** Creates a new ShootAtRPMCmdSeq. */
  public ShootAtRPMCmdSeq(double rpm, double armAngle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(
        new BangBangShooterCmd(
            rpm),
        new RunIntakeWhenShooterAtRPMCmd(
            rpm),
        new HoldArmInPositionCmd(
            armAngle)));
  }
}
