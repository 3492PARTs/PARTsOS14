// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Drive.PIDDriveCmd;
import frc.robot.commands.Intake.RunIntakePhotoEyeCmd;
import frc.robot.util.PIDValues;

public class AutoMoveForward extends SequentialCommandGroup {
  /** Creates a new MoveForward.*/
  public AutoMoveForward() {
    // Auto move forward 90 inches.
    addCommands(new PIDDriveCmd(Units.inchesToMeters(108)));
    /* 
    addCommands(new ZeroArmCmdSeq(),
        new ProfiledPivotArmCmd(Constants.Arm.SPEAKER),
        new ProfiledPivotArmCmd(Constants.Arm.GROUND),
        new RunIntakePhotoEyeCmd(Constants.Intake.INTAKE_SPEED),
        new PIDDriveCmd(Units.inchesToMeters(36)));
        */
  }
}
