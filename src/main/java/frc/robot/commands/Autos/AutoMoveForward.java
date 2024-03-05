// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveDistanceCmd;

public class AutoMoveForward extends SequentialCommandGroup {
  /** Creates a new MoveForward.*/
  public AutoMoveForward() {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new DriveDistanceCmd(Units.inchesToMeters(90)));
  }
}
