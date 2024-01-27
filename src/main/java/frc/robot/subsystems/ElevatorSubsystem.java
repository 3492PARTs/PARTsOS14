// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.equation.Variable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  enum pivotPos {
    GROUND,
    HOME,
    SPEAKER,
    AMP
  }


  public ElevatorSubsystem() {


  }

  public double getAngle() {
    // TODO: Replace with real later.
    double result = 0.0;
    return result;
  }

  public double getTargetPivotPos(pivotPos pivPos) {
    switch (pivPos) {
      case GROUND:
      //TODO: Value.
        break;
      case HOME:
      //TODO: Value.
        break;
      case SPEAKER:
      //TODO: Value.
        break;
      case AMP:
      //TODO: Value.
        break;
    
      default:
        break;
    }
    // TODO: Make safe value.
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
