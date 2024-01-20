// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  static CANSparkMax leftLeader = new CANSparkMax(10, MotorType.kBrushless);
  static CANSparkMax left2 = new CANSparkMax(11, MotorType.kBrushless);

  static CANSparkMax rightLeader = new CANSparkMax(10, MotorType.kBrushless);
  static CANSparkMax right2 = new CANSparkMax(10, MotorType.kBrushless);

  XboxController driveController = new XboxController(0);
  
  private DriveTrain() {

    left2.follow(leftLeader);
    right2.follow(rightLeader);

    rightLeader.setInverted(true);

    DifferentialDrive m_drive = new DifferentialDrive(leftLeader, rightLeader);

    //leftLeader.setOpenLoopRampRate(.85);
    //rightLeader.setOpenLoopRampRate(.85);

    m_drive.arcadeDrive(driveController.getLeftY(), driveController.getLeftX());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }
}
