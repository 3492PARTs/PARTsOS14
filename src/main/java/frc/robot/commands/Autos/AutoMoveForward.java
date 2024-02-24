// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.TimeDriveCmd;
import frc.robot.commands.Drive.PIDdriveCmd;
import frc.robot.subsystems.PIDValues;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveForward extends SequentialCommandGroup {
  /** Creates a new MoveForward. */
  public AutoMoveForward() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TimeDriveCmd(3));
    // addCommands(new PIDdrive(new PIDValues(1.0, 0.0, 0.0),
    // Units.inchesToMeters(36)));
    // addCommands(new PIDValues(0.0, 0.0, 0.0), new
    // PIDdrive(Units.inchesToMeters(36)));
  }
}
