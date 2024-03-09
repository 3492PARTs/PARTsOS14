// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.PIDdriveCmd;
import frc.robot.util.PIDValues;

public class AutoMoveForward extends SequentialCommandGroup {
  /** Creates a new MoveForward.*/
  public AutoMoveForward() {
    // Auto move forward 90 inches.
    addCommands(new PIDdriveCmd(new PIDValues(9.0, 0, 0.5), Units.inchesToMeters(72)));
  }
}
