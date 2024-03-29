// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

  private static Intake intakeInstance;

  private static TalonSRX intakeMotor;

  private static DigitalInput photoEye;

  /** Creates a new Intake. */
  public Intake() {
    if (Robot.isReal())
      photoEye = new DigitalInput(Constants.Intake.PHOTOEYE);
    intakeMotor = new TalonSRX(Constants.Intake.INTAKE_MOTOR);
  }

  public static Intake getInstance() {
    // If instance is null, then make a new instance.
    if (intakeInstance == null) {
      intakeInstance = new Intake();
    }

    return intakeInstance;
  }

  // sends current to intake
  public void runIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public DoubleSupplier intakeCurrentSupplier() {
    DoubleSupplier s = () -> intakeMotor.getSupplyCurrent();
    return s;
  }

  // methods for note detection using photoEyes
  public Boolean hasNote() {
    return !photoEye.get();
  }

  public BooleanSupplier hasNoteSupplier() {
    BooleanSupplier s = () -> hasNote();
    return s;
  }

  public BooleanSupplier doesNotHaveNoteSupplier() {
    BooleanSupplier s = () -> !hasNote();
    return s;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
