// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.Arm.ArmToPositionCmd;
import frc.robot.commands.Arm.ZeroPivotEncoders;
import frc.robot.commands.Drive.MoveForward;
import frc.robot.commands.IntakeShoot.IntakeShootCmd;
import frc.robot.commands.IntakeShoot.RunIntakeCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();

  //private final SlewRateLimiter speedLimiter = new SlewRateLimiter(1, -1, 0);

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

    
    //Operator Triggers and Axis

    arm.setDefaultCommand(
      new RunCommand(() -> arm.setPivotSpeed(
        operatorController.getRightY()),
        arm)
    );

      //operatorController.leftTrigger(.4).whileTrue(new RunIntakeCmd(-1));
      //operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));
     // operatorController.b().whileTrue(new IntakeShootCmd());
     // operatorController.a().whileTrue(new ZeroPivotEncoders());
    

    shooter.setDefaultCommand(
      new RunCommand(() -> shooter.runShooter(
        operatorController.getRightTriggerAxis()),
        shooter)
    );

    //Operator Buttons
    //operatorController.x().onTrue(new ArmToPositionCmd(75));

    //SysID
    operatorController.a().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    operatorController.b().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    operatorController.x().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    operatorController.y().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public void displaySmartDashboard() {
    SmartDashboard.putNumber("left Drive Distance", DriveTrain.getInstance().leftDistance());
    SmartDashboard.putNumber("right Drive Distance", DriveTrain.getInstance().rightDistance());
    SmartDashboard.putNumber("left Drive Encoder", DriveTrain.getInstance().leftEncoderPosition());
    SmartDashboard.putNumber("right Drive Encoder", DriveTrain.getInstance().rightEncoderPosition());
    SmartDashboard.putNumber("left Pivot Encoder", Arm.getInstance().leftPivotEncoderPosition());
    SmartDashboard.putNumber("right Pivot Encoder", Arm.getInstance().rightPivotEncoderPosition());
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