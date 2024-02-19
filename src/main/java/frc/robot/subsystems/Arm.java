// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private static Arm armInstance;
  /** Creates a new ArmSubsystem. */

  static CANSparkMax pivotLeftMotor;
  static CANSparkMax pivotRightMotor;
  
  SparkPIDController pivotLeftController;
  SparkPIDController pivotRightController;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  TrapezoidProfile.Constraints ArmConstraints;

  ArmFeedforward armFeedForward;

  //TODO: tune PID values
  double kP = 0.1;
  double kI = 0.0;
  double kD = 0.0;

  double kS = 0.0;
  double kG = 0.0;
  double kV = 0.0;

  //TODO: tune PID values
  //PID for only armfeedforward
  PIDController velocityPID = new PIDController(1.05, 8, .001); 


  //TODO: update gear ratio 
  double pivotGearRatio = Constants.Arm.PIVOT_GEAR_RATIO;

  //SysID routine
  public final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (voltage) -> armInstance.driveMotorVolts(voltage.in(Volts)),
       null,
      armInstance
    )
  );

  public Arm() {

    //TODO: change angdeg higher later (faster the better)
    ArmConstraints = new TrapezoidProfile.Constraints(Math.toRadians(30), Math.toRadians(6));

    //ks: overcomes static friction
    //kg: voltage needed to maintain speed
    //kv: voltage needed to accelerate
    armFeedForward = new ArmFeedforward(0.039224, 0.31122, 0.012932);

    pivotLeftMotor= new CANSparkMax(Constants.Arm.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(Constants.Arm.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

    pivotLeftMotor.setInverted(false);
    pivotRightMotor.setInverted(true);

    pivotLeftController = pivotLeftMotor.getPIDController();
    pivotRightController = pivotRightMotor.getPIDController();

    //TODO: tune these values
    pivotLeftController.setP(kP);
    pivotLeftController.setI(kI);
    pivotLeftController.setD(kD);
    pivotLeftController.setOutputRange(kI,kD);

    pivotRightController.setP(kP);
    pivotRightController.setI(kI);
    pivotRightController.setD(kD);
    pivotRightController.setOutputRange(kI, kD);

    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

    pivotLeftMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);
    pivotRightMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);

    Shuffleboard.getTab("debug").addNumber("arm angle", getAngleSupplier());
    Shuffleboard.getTab("debug").addNumber("arm angular velocity", getAnglularVelocitySupplier());
  }


  public static Arm getInstance() {
    // If instance is null, then make a new instance.
    if (armInstance == null) { armInstance = new Arm(); }
    return armInstance;
  }

  public TrapezoidProfile.Constraints getConstraints() {
    return ArmConstraints;
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(Math.toRadians(getAngle()), Math.toRadians(getAnglularVelocitySupplier().getAsDouble()));
  }

  //calculates the voltage the arm feedforward needs
  public double calcOutputVoltage(double velocity) {
    double output = (armFeedForward.calculate(Math.toRadians((getAngle())), velocity) + velocityPID.calculate(Math.toRadians(getRotationRate()), velocity));
    return output;
  }

  public void driveMotorVolts(double volts) {
    pivotLeftMotor.setVoltage(volts);
    pivotRightMotor.setVoltage(volts);
  }

  public double getAngle() {
    return 360 * pivotLeftMotor.getEncoder().getPosition() / pivotGearRatio;
  }

  public DoubleSupplier getAngleSupplier() {
    DoubleSupplier s = () -> getAngle();
    return s;
  }

  public double rightPivotEncoderPosition() {
    return pivotRightMotor.getEncoder().getPosition();
  }

  public double leftPivotEncoderPosition() {
    return pivotLeftMotor.getEncoder().getPosition();
  }

  //TODO: check number 60
  public double getRotationRate() {
    return 360 * pivotLeftMotor.getEncoder().getVelocity() / (pivotGearRatio * 60);

  }

  public DoubleSupplier getAnglularVelocitySupplier() {
    DoubleSupplier s = () -> getRotationRate();
    return s;
  }

  //added method in robotinit() in Robot.java
  public void zeroPivotEncoders() {
    pivotLeftMotor.getEncoder().setPosition(0);
    pivotRightMotor.getEncoder().setPosition(0);
  }


  public void setPivotSpeed(double speed) {
    speed = Math.abs(speed) > 0.1? speed:0;
    pivotLeftMotor.set(speed/2);
    pivotRightMotor.set(speed/2);
  }

  public void setPivotPoint(double position) {
    pivotLeftController.setReference(position, ControlType.kPosition);
    pivotRightController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Shuffleboard.update();
  }
}
