// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.HoldArmInPosition;
import frc.robot.commands.Arm.ProfiledPivotArm;
import frc.robot.commands.Arm.ZeroPivotEncoders;
import frc.robot.commands.Autos.MoveForward;
import frc.robot.commands.IntakeShoot.RunIntakeCmd;
import frc.robot.commands.IntakeShoot.ShootCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = DriveTrain.getInstance();
  private final Arm arm = Arm.getInstance();
  // Supressing this for now because I know we're gonna use it but this is driving me mad.
  @SuppressWarnings("unused")
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();

  HoldArmInPosition holdArmInPosition = null;

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("choose auto mode", autoChooser);
    autoChooser.addOption("Move Forward 3 seconds", new MoveForward());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    driveTrain.setDefaultCommand(
        new RunCommand(() -> driveTrain.driveArcade(
          driveController.getLeftY(), 
          driveController.getRightX()),
          driveTrain)
    );


    arm.setDefaultCommand(
      new RunCommand(() -> {
        if (Math.abs(operatorController.getRightY()) > .1) {
          if (holdArmInPosition != null) {
            //System.out.println("UNschedule");
            //holdArmInPosition.cancel();
            holdArmInPosition = null;
          }
          arm.setPivotSpeed(operatorController.getRightY());
        }

        else {
          if (holdArmInPosition == null) {
            //System.out.println("schedule");
            //System.out.println("angle " + arm.getAngle());
            arm.setPivotSpeed(0);
            holdArmInPosition = new HoldArmInPosition(arm.getAngle());
            holdArmInPosition.schedule();   
          }
        }
      },
      arm)
  );

    /* 
    shooter.setDefaultCommand(
      new RunCommand(() -> shooter.runShooter(
        operatorController.getRightTriggerAxis() > 0? 1:0),
        shooter)
    );
    */

    //operatorController.rightTrigger(.4).whileTrue(new IntakeShootCmd(-.75));
    //operatorController.rightBumper().whileTrue(new IntakeShootCmd())

    operatorController.rightTrigger(.4).whileTrue(new ShootCmd());
    

    //Operator Buttons
    operatorController.x().onTrue(new ProfiledPivotArm(70, 2.7, 0.0, 0.0));
    operatorController.b().onTrue(new ProfiledPivotArm(30, 2.7, 0.0, 0.0));
    operatorController.y().onTrue(new ProfiledPivotArm(-5.09, 3.0, 0.3, 0.0));

    //operatorController.rightTrigger(.4).whileTrue(new ShootCmd());

    operatorController.leftTrigger(.4).whileTrue(new RunIntakeCmd(-.75));
    operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));
    //operatorController.b().whileTrue(new IntakeShootCmd());
    operatorController.a().whileTrue(new ZeroPivotEncoders());

    

    //SysID
    /* 
    operatorController.a().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    operatorController.b().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    operatorController.x().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    operatorController.y().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    */
  }

  public void displaySmartDashboard() {
    SmartDashboard.putNumber("left Drive Distance", DriveTrain.getInstance().leftDistance());
    SmartDashboard.putNumber("right Drive Distance", DriveTrain.getInstance().rightDistance());
    SmartDashboard.putNumber("left Drive Encoder", DriveTrain.getInstance().leftEncoderPosition());
    SmartDashboard.putNumber("right Drive Encoder", DriveTrain.getInstance().rightEncoderPosition());
    SmartDashboard.putNumber("left Pivot Encoder", Arm.getInstance().leftPivotEncoderPosition());
    SmartDashboard.putNumber("right Pivot Encoder", Arm.getInstance().rightPivotEncoderPosition());
    SmartDashboard.putNumber("current voltage", Shooter.shooterLeftLeader.getBusVoltage());
  }


  public CommandXboxController getOperatorController () {
    return operatorController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}