// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Middle.StartCommands;

import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos.Middle.AutoThreeNoteMiddle;

public class StartAutoThreeNoteMiddle extends Command {
  private int red = 1;
  private GenericEntry ampOrEmpty;
  private GenericEntry speakerOrAmp;

  private boolean ampOrEmptyBool;
  private boolean speakerOrAmpBool;

  /** Creates a new StartAutoCmd. */
  public StartAutoThreeNoteMiddle(GenericEntry ampOrEmpty, GenericEntry speakerOrAmp) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ampOrEmpty = ampOrEmpty;
    this.speakerOrAmp = speakerOrAmp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        red = 1;
      }
      if (ally.get() == Alliance.Blue) {
        red = -1;
      }
    }

    ampOrEmptyBool = ampOrEmpty.getBoolean(false);
    speakerOrAmpBool = speakerOrAmp.getBoolean(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(new AutoThreeNoteMiddle(red, ampOrEmptyBool, speakerOrAmpBool));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
