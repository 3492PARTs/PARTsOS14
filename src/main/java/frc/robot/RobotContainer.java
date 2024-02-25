// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmToPositionCmd;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ZeroPivotEncodersCmd;
import frc.robot.commands.Autos.AutoMoveForward;
import frc.robot.commands.IntakeShoot.RunIntakeCmd;
import frc.robot.commands.IntakeShoot.ShootInAmpCmd;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Arm.HoldArmInPositionCmd;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain driveTrain = DriveTrain.getInstance();
  private final Arm arm = Arm.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private static RobotContainer robotContainerInstance;

  // Drive controls drivetrain, operator controls arm, intake, and shooter.
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // SmartDashboard chooser for auto tasks.
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("choose auto mode", autoChooser);
    autoChooser.addOption("Move Forward", new AutoMoveForward());
  }

  public static RobotContainer getInstance() {
    // If instance is null, then make a new instance.
    if (robotContainerInstance == null) {
      robotContainerInstance = new RobotContainer();
    }
    return robotContainerInstance;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {

    driveTrain.setDefaultCommand(
        new RunCommand(() -> driveTrain.driveArcade(
            driveController.getLeftY(),
            driveController.getRightX()),
            driveTrain));

    driveController.a().whileTrue(new ZeroPivotEncodersCmd());

    arm.setDefaultCommand(
        new RunCommand(() -> {
          if (Math.abs(operatorController.getRightY()) > .1) {
            arm.setPivotSpeed(operatorController.getRightY());
          } else {
            arm.setPivotSpeed(0);
            new HoldArmInPositionCmd(arm.getAngle()).schedule();
          }
        },
            arm));

    // arm.setDefaultCommand(new HoldArmInPositionCmd());

    operatorController.x().onTrue(new ArmToPositionCmd(Constants.Arm.GROUND)); // ground
    operatorController.y().onTrue(new ArmToPositionCmd(Constants.Arm.SPEAKER)); // speaker
    operatorController.b().onTrue(new ArmToPositionCmd(Constants.Arm.HOME)); // home
    operatorController.a().onTrue(new ArmToPositionCmd(Constants.Arm.AMP)); // amp

    operatorController.rightTrigger(.1).whileTrue(new ShootInSpeakerCmd());
    operatorController.rightBumper().whileTrue(new ShootInAmpCmd());

    operatorController.leftTrigger(.1).whileTrue(new RunIntakeCmd(-.75));
    operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));

    // Profiled Pivot Buttons
    /*
     * operatorController.x().onTrue(new ProfiledPivotArmCmd(80, 2.7, 0, 0)); //
     * ground
     * operatorController.y().onTrue(new ProfiledPivotArmCmd(42.8, 3.0, 0.0, 0.0));
     * // speaker
     * operatorController.b().onTrue(new ProfiledPivotArmCmd(30, 2.7, 0.0, 0.0)); //
     * home
     * operatorController.a().onTrue(new ProfiledPivotArmCmd(-5.09, 3.0, 0.3, 0.0));
     */

    // SysID
    if (Constants.Arm.SYSID) {
      operatorController.a().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
      operatorController.b().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
      operatorController.x().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
      operatorController.y().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  public void displaySmartDashboard() {
    // Drive
    SmartDashboard.putNumber("left Drive Distance", DriveTrain.getInstance().leftDistance());
    SmartDashboard.putNumber("right Drive Distance", DriveTrain.getInstance().rightDistance());
    // SmartDashboard.putData("zero Drive Encoder", new ZeroDriveEncodersCmd());
    // SmartDashboard.putNumber("left Drive Encoder",
    // DriveTrain.getInstance().leftEncoderPosition());
    // SmartDashboard.putNumber("right Drive Encoder",
    // DriveTrain.getInstance().rightEncoderPosition());

    // Pivot
    // SmartDashboard.putData("zero Pivot Encoders", new ZeroPivotEncodersCmd());
    SmartDashboard.putNumber("left Pivot Encoder", Arm.getInstance().leftPivotEncoderPosition());
    SmartDashboard.putNumber("right Pivot Encoder", Arm.getInstance().rightPivotEncoderPosition());

    // Shooter
    SmartDashboard.putNumber("shooter RPM", Shooter.getInstance().getShooterRPM());

    // PhotoEye
    SmartDashboard.putBoolean("HAS NOTE", Intake.getInstance().hasNote());
  }

  public CommandXboxController getOperatorController() {
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