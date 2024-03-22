// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase {

  private static Arm armInstance;
  /** Creates a new ArmSubsystem. */

  static CANSparkMax pivotLeftMotor;
  static CANSparkMax pivotRightMotor;
  DigitalInput armLimit = new DigitalInput(Constants.Arm.L_SWITCH_PORT);

  SparkPIDController pivotLeftController;
  SparkPIDController pivotRightController;

  TrapezoidProfile.Constraints ArmConstraints;

  ArmFeedforward armFeedForward;

  double kS = 0.02204; //0.21092; // .38107
  double kV = 0.15682;//0.11019; // .10239
  double kG = 0.65433; //0.02501; // .02501
  double kA = 0.027191;//0.021335;

  double pivotGearRatio = Constants.Arm.PIVOT_GEAR_RATIO;

  // SysID routine  
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  RelativeEncoder alternateLeftEncoder;
  RelativeEncoder alternateRightEncoder;

  public static double downSpeed = -.25;
  public static double upSpeed = .25;

  public boolean armLimitBuffer = false;

  public SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (voltage) -> this.driveMotorVolts(voltage.in(Volts)),

          log -> {
            // Record a frame for the shooter motor.
            log.motor("pivotarm")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        getAverageVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(
                    ((alternateLeftEncoder.getPosition())),
                    Rotations))
                .angularVelocity(
                    m_velocity.mut_replace(getRPS(), RotationsPerSecond));
          },
          this));

  public Arm() {

    // TODO: change angdeg higher later (faster the better)
    ArmConstraints = new TrapezoidProfile.Constraints(Math.toRadians(50), Math.toRadians(55));

    // ks: overcomes static friction
    // kg: voltage needed to maintain speed
    // kv: voltage needed to accelerate
    armFeedForward = new ArmFeedforward(kS, kG, kV, kA);

    pivotLeftMotor = new CANSparkMax(Constants.Arm.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(Constants.Arm.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

    pivotLeftMotor.setInverted(false);
    pivotRightMotor.setInverted(true);

    pivotLeftController = pivotLeftMotor.getPIDController();
    pivotRightController = pivotRightMotor.getPIDController();

    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

    alternateLeftEncoder = pivotLeftMotor.getAlternateEncoder(8192);
    alternateRightEncoder = pivotRightMotor.getAlternateEncoder(8192);

    pivotLeftMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);
    pivotRightMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);

    Shuffleboard.getTab("debug").addNumber("arm angle", getAngleSupplier());
    Shuffleboard.getTab("debug").addNumber("arm angle g", getAngleSupplier());
    // Shuffleboard.getTab("debug").addNumber("arm angular velocity",
    // getAnglularVelocitySupplier());
  }

  public static Arm getInstance() {
    // If instance is null, then make a new instance.
    if (armInstance == null) {
      armInstance = new Arm();
    }
    return armInstance;
  }

  // Setting calculations
  public void setPivotSpeed(double speed) {
    // speed = Math.abs(speed) > 0.1 ? speed : 0;
    pivotLeftMotor.set(speed);
    pivotRightMotor.set(speed);
  }

  public void setPivotPoint(double position) {
    pivotLeftController.setReference(position, ControlType.kPosition);
    pivotRightController.setReference(position, ControlType.kPosition);
  }

  // Trapezoid Profiling
  public TrapezoidProfile.Constraints getConstraints() {
    return ArmConstraints;
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(Math.toRadians(getAngle()),
        Math.toRadians(getAnglularVelocitySupplier().getAsDouble()));
  }

  // voltage calculations
  public double calcOutputVoltage(double velocity) {
    double output = (armFeedForward.calculate(Math.toRadians((getAngle())), velocity));
    // SmartDashboard.putNumber("calcOutputVoltage", output);
    return output;
  }

  public void driveMotorVolts(double volts) {
    pivotLeftMotor.setVoltage(volts);
    pivotRightMotor.setVoltage(volts);
  }

  // Angle calculations
  public double getAngle() {
    //return 360 * pivotLeftMotor.getEncoder().getPosition() / pivotGearRatio;
    return getAlternateEncoderPosition();
  }

  public double getAlternateEncoderPosition() {
    //return alternateLeftEncoder.getPosition();
    return (alternateLeftEncoder.getPosition() * 360) * -1;
  }

  public double getAlternateEncoderCPR() {
    return alternateLeftEncoder.getCountsPerRevolution();
  }

  public DoubleSupplier getAngleSupplier() {
    DoubleSupplier s = () -> getAngle();
    return s;
  }

  // Rotation calculations
  public double getRotationRate() {
    return 360 * pivotLeftMotor.getEncoder().getVelocity() / (60);
  }

  public DoubleSupplier getAnglularVelocitySupplier() {
    DoubleSupplier s = () -> getRotationRate();
    return s;
  }

  // Encoder calculations
  public double rightPivotEncoderPosition() {
    return pivotRightMotor.getEncoder().getPosition();
  }

  public double leftPivotEncoderPosition() {
    return pivotLeftMotor.getEncoder().getPosition();
  }

  public void zeroPivotEncoders() {
    pivotLeftMotor.getEncoder().setPosition(0);
    pivotRightMotor.getEncoder().setPosition(0);

    alternateLeftEncoder.setPosition(0);
    alternateRightEncoder.setPosition(0);
  }

  // SysID methods
  public double getAverageVoltage() {
    double averageVolts = ((pivotLeftMotor.getAppliedOutput() * pivotLeftMotor.getBusVoltage())
        + (pivotRightMotor.getAppliedOutput() * pivotRightMotor.getBusVoltage())) / 2;
    return averageVolts;
  }

  public double getRPS() {
    return ((pivotLeftMotor.getEncoder().getVelocity() / 60) + (pivotLeftMotor.getEncoder().getVelocity() / 60)) / 2;
  }

  public boolean getSwitch() {
    if (!armLimit.get()) {
      /*if (!armLimitBuffer) {
        setPivotSpeed(0);
      }
      */
      return true;
    } else {
      //armLimitBuffer = false;
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
