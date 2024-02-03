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

    pivotLeftLeader.setInverted(true);
    pivotRightFollower.setInverted(false);

    pivotController = pivotLeftLeader.getPIDController();

    pivotLeftLeader.setIdleMode(IdleMode.kBrake);
    pivotRightFollower.setIdleMode(IdleMode.kBrake);

  }

  public static Arm getInstance() {
    // If instance is null, then make a new instance.
    if (armInstance == null) { armInstance = new Arm(); }
    return armInstance;
  }

  // Enum for arm positions.
  public enum PivotPos {
    GROUND(0),
    SPEAKER(0),
    AMP(0);

    int value;
    PivotPos(int value) {
      this.value = value;
    }
    public int getValue() {
        return value;
    }
  }

  public double getAngle() {
    return 360 * pivotLeftLeader.getEncoder().getPosition() / pivotGearRatio;
    
  }

  public void setPivotSpeed(double speed) {
    pivotLeftLeader.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
