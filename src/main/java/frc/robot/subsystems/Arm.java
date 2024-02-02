// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO: implement feedforward controls
public class Arm extends SubsystemBase {

  private static Arm armInstance;
  /** Creates a new ArmSubsystem. */

  static CANSparkMax pivotLeftLeader;
  static CANSparkMax pivotRightFollower;
  
  SparkPIDController pivotController;

  //TODO: update gear ratio 
  double pivotGearRatio = Constants.Arm.PIVOT_GEAR_RATIO;

  public Arm() {
    pivotLeftLeader= new CANSparkMax(Constants.Arm.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotRightFollower = new CANSparkMax(Constants.Arm.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

    pivotRightFollower.follow(pivotLeftLeader);

    pivotLeftLeader.setInverted(false);
    pivotRightFollower.setInverted(true);

    pivotController = pivotLeftLeader.getPIDController();

    pivotLeftLeader.setIdleMode(IdleMode.kBrake);
    pivotRightFollower.setIdleMode(IdleMode.kBrake);

  }

  public static Arm getInstance() {
    // If instance is null, then make a new instance.
    if (armInstance == null) { armInstance = new Arm(); }
    return armInstance;
  }

  public enum PivotPos {
    GROUND,
    SPEAKER,
    AMP
  }

  public double getAngle() {
    return 360 * pivotLeftLeader.getEncoder().getPosition() / pivotGearRatio;
    
  }

  public void setPivotSpeed(double speed) {
    pivotLeftLeader.set(speed);
  }

  public double getTargetPivotPos(PivotPos pivPos) {
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
