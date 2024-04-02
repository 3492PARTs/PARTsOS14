// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private static Arm armInstance;

  private static CANSparkMax pivotLeftMotor;
  private static CANSparkMax pivotRightMotor;
  private static DigitalInput armLimit = new DigitalInput(Constants.Arm.L_SWITCH_PORT);

  public TrapezoidProfile.Constraints ArmConstraints;

  public ArmFeedforward armFeedForward;

  // SysID routine  
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private static RelativeEncoder alternateLeftEncoder;
  private static RelativeEncoder alternateRightEncoder;

  public SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (voltage) -> this.setVolts(voltage.in(Volts)),

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
    ArmConstraints = new TrapezoidProfile.Constraints(Math.toRadians(100), Math.toRadians(100));

    // ks: overcomes static friction
    // kg: voltage needed to maintain speed
    // kv: voltage needed to accelerate
    armFeedForward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV, Constants.Arm.kA);

    pivotLeftMotor = new CANSparkMax(Constants.Arm.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(Constants.Arm.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);

    pivotLeftMotor.setInverted(false);
    pivotRightMotor.setInverted(true);

    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

    alternateLeftEncoder = pivotLeftMotor.getAlternateEncoder(8192);
    alternateRightEncoder = pivotRightMotor.getAlternateEncoder(8192);

    pivotLeftMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);
    pivotRightMotor.setOpenLoopRampRate(Constants.Arm.OPEN_LOOP_RATE);
  }

  public static Arm getInstance() {
    // If instance is null, then make a new instance.
    if (armInstance == null) {
      armInstance = new Arm();
    }
    return armInstance;
  }

  public void setSpeed(double speed) {
    pivotLeftMotor.set(speed);
    pivotRightMotor.set(speed);
  }

  public void setVolts(double volts) {
    pivotLeftMotor.setVoltage(volts);
    pivotRightMotor.setVoltage(volts);
  }

  // Angle calculations
  public double getAngle() {
    // old way using motor encoder return 360 * pivotLeftMotor.getEncoder().getPosition() / pivotGearRatio;

    double left = alternateLeftEncoder.getPosition();
    double right = alternateRightEncoder.getPosition();
    return ((Math.abs(left) > Math.abs(right) ? left : right) * 360) * -1; // CPR * 360 to convert to angle * -1 to flip the angle orientation
  }

  public DoubleSupplier getAngleSupplier() {
    DoubleSupplier s = () -> getAngle();
    return s;
  }

  // Rotation calculations
  public double getRotationRate() {
    //TODO: need to change to alternate encoders

    double left = pivotLeftMotor.getEncoder().getVelocity();
    double right = pivotRightMotor.getEncoder().getVelocity();
    return 360 * (Math.abs(left) > Math.abs(right) ? left : right) / (60);
  }

  public DoubleSupplier getRotationRateSupplier() {
    DoubleSupplier s = () -> getRotationRate();
    return s;
  }

  public void zeroPivotEncoders() {
    pivotLeftMotor.getEncoder().setPosition(0);
    pivotRightMotor.getEncoder().setPosition(0);

    alternateLeftEncoder.setPosition(0);
    alternateRightEncoder.setPosition(0);
  }

  public boolean getLimitSwitch() {
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

  public BooleanSupplier getLimitSwitchSupplier() {
    return this::getLimitSwitch;
  }

  // Trapezoid Profiling
  public TrapezoidProfile.Constraints getConstraints() {
    return ArmConstraints;
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(Math.toRadians(getAngle()),
        Math.toRadians(getRotationRateSupplier().getAsDouble()));
  }

  // voltage calculations
  public double calcOutputVoltage(double velocity) {
    double output = (armFeedForward.calculate(Math.toRadians((getAngle())), velocity));
    // SmartDashboard.putNumber("calcOutputVoltage", output);
    return output;
  }

  // SysID methods
  public double getAverageVoltage() {
    double averageVolts = ((pivotLeftMotor.getAppliedOutput() * pivotLeftMotor.getBusVoltage())
        + (pivotRightMotor.getAppliedOutput() * pivotRightMotor.getBusVoltage())) / 2;
    return averageVolts;
  }

  @Deprecated
  public double getRPS() {
    //TODO: Replace with alternate encoder

    double left = pivotLeftMotor.getEncoder().getVelocity() / 60;
    double right = pivotLeftMotor.getEncoder().getVelocity() / 60;

    return Math.abs(left) > Math.abs(right) ? left : right;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Test this works
  /* 
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish the solenoid state to telemetry.
    builder.addBooleanProperty("On Ground", this.getLimitSwitchSupplier(), null);
    builder.addDoubleProperty("Current Arm Angle", this.getAngleSupplier(), null);
  }
  */
}
