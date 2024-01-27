// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private static Arm armInstance;
  /** Creates a new ArmSubsystem. */

  static CANSparkMax pivotLeftMotor;
  static CANSparkMax pivotRightMotor;
  

  public Arm() {
    pivotLeftMotor= new CANSparkMax(Constants.Arm.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(Constants.Arm.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);
  }

  public static Arm getInstance() {
    // If instance is null, then make a new instance.
    if (armInstance == null) { armInstance = new Arm(); }
    return armInstance;
  }

  enum pivotPos {
    GROUND,
    SPEAKER,
    AMP
  }

  public double getAngle() {
    // TODO: Replace with real later.
    double result = 0.0;
    return result;
  }

  public void setPivotSpeed(double speed) {

  }

  public double getTargetPivotPos(pivotPos pivPos) {
    switch (pivPos) {
      case GROUND:
      //TODO: Value.
        break;
      case SPEAKER:
      //TODO: Value.
        break;
      case AMP:
      //TODO: Value.
        break;
    
      default:
      //TODO: HOME
        break;
    }
    //TODO: Make safe value.
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
