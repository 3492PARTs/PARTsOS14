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

  static CANSparkMax pivotLeftMotor;
  static CANSparkMax pivotRightMotor;
  
  SparkPIDController pivotController;

  //TODO: update gear ratio 
  double pivotGearRatio = Constants.Arm.PIVOT_GEAR_RATIO;

  public Arm() {
    pivotLeftMotor= new CANSparkMax(Constants.Arm.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(Constants.Arm.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

    pivotLeftMotor.setInverted(true);
    pivotRightMotor.setInverted(false);

    pivotController = pivotLeftMotor.getPIDController();

    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

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
    return 360 * pivotLeftMotor.getEncoder().getPosition() / pivotGearRatio;
    
  }

  public void setPivotSpeed(double speed) {
    pivotLeftMotor.set(speed);
    pivotRightMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
