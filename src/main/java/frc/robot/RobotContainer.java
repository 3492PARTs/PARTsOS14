// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ZeroPivotEncodersCmd;
import frc.robot.commands.Autos.AutoMoveForward;
import frc.robot.commands.Autos.AutoOneNoteLeftPos;
import frc.robot.commands.Autos.AutoOneNoteMiddlePos;
import frc.robot.commands.Autos.AutoOneNoteRightPos;
import frc.robot.commands.Autos.AutoTwoNoteLeftPos;
import frc.robot.commands.Autos.AutoTwoNoteMiddlePos;
import frc.robot.commands.Autos.AutoTwoNoteRightPos;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

  // Drive controls drivetrain, operator controls arm, intake, and shooter.
  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  // SmartDashboard chooser for auto tasks.
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the trigger bindings
    // configureBindings();

    SmartDashboard.putData("choose auto mode", autoChooser);
    autoChooser.addOption("Move Forward", new AutoMoveForward());
    autoChooser.addOption("One Note Middle", new AutoOneNoteMiddlePos());
    autoChooser.addOption("One Note Right ", new AutoOneNoteRightPos());
    autoChooser.addOption("One Note Left", new AutoOneNoteLeftPos());
    autoChooser.addOption("Two Note Middle", new AutoTwoNoteMiddlePos());
    autoChooser.addOption("Two Note Right", new AutoTwoNoteRightPos());
    autoChooser.addOption("Two Note Left", new AutoTwoNoteLeftPos());
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

  public void configureBindings() {

    //* Default commands */
    driveTrain.setDefaultCommand(
        new RunCommand(() -> driveTrain.driveArcade(
            driveController.getLeftY(),
            driveController.getRightX()),
            driveTrain));

    driveController.a().whileTrue(new ZeroPivotEncodersCmd());

    arm.setDefaultCommand(
        new RunCommand(() -> {
          // manual control
          if (Math.abs(operatorController.getRightY()) > .1) {
            /*
            TODO: Test before implementation
            
            if (arm.getAngle() > Constants.Arm.LOWER_BOUND) {
              if (operatorController.getRightY() > 0)
                arm.setPivotSpeed(operatorController.getRightY());
              else
                arm.setPivotSpeed(0);
            }
            else if (arm.getAngle() < Constants.Arm.UPPER_BOUND) {
              if (operatorController.getRightY() < 0)
                arm.setPivotSpeed(operatorController.getRightY());
              else
                arm.setPivotSpeed(0);
            }
            else
              arm.setPivotSpeed(operatorController.getRightY());
            */

            arm.setPivotSpeed(operatorController.getRightY());
          }
          // hold arm in current position
          else {
            // TODO: Idea to help the arm get a more consistent stopping point angle. We schedule a command to stop the arm then a wait then the hold in position.
            arm.setPivotSpeed(0);
            new HoldArmInPositionCmd(arm.getAngle()).schedule();
          }
        },
            arm));

    //* Controller Bindings */

    operatorController.x().onTrue(new ArmToPositionTeleopCmd(Constants.Arm.GROUND)); // ground
    operatorController.y().onTrue(new ArmToPositionTeleopCmd(Constants.Arm.SPEAKER)); // speaker
    operatorController.b().onTrue(new ArmToPositionTeleopCmd(Constants.Arm.HOME)); // home
    operatorController.a().onTrue(new ArmToPositionTeleopCmd(Constants.Arm.AMP)); // amp

    operatorController.rightTrigger(.1).whileTrue(new ShootInSpeakerCmd());
    operatorController.rightBumper().onTrue(new ShootInAmpCmd());

    operatorController.leftTrigger(.1).whileTrue(new RunIntakeCmd(-.75));
    operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));

    // Profiled Pivot Buttons
    /*
      operatorController.x().onTrue(new ProfiledPivotArmCmd(80, 2.7, 0, 0)); //
      // ground
      operatorController.y().onTrue(new ProfiledPivotArmCmd(42.8, 3.0, 0.0, 0.0));
      // speaker
      operatorController.b().onTrue(new ProfiledPivotArmCmd(30, 2.7, 0.0, 0.0)); //
      // home
      operatorController.a().onTrue(new ProfiledPivotArmCmd(-5.09, 3.0, 0.3, 0.0));
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
    SmartDashboard.putNumber("left Drive Distance", driveTrain.leftDistance());
    SmartDashboard.putNumber("right Drive Distance", driveTrain.rightDistance());
    // SmartDashboard.putData("zero Drive Encoder", new ZeroDriveEncodersCmd());
    // SmartDashboard.putNumber("left Drive Encoder",
    // DriveTrain.getInstance().leftEncoderPosition());
    // SmartDashboard.putNumber("right Drive Encoder",
    // DriveTrain.getInstance().rightEncoderPosition());

    // Pivot
    // SmartDashboard.putData("zero Pivot Encoders", new ZeroPivotEncodersCmd());
    // SmartDashboard.putNumber("left Pivot Encoder",
    // Arm.getInstance().leftPivotEncoderPosition());
    // SmartDashboard.putNumber("right Pivot Encoder",
    // Arm.getInstance().rightPivotEncoderPosition());

    // Shooter
    SmartDashboard.putNumber("shooter RPM", shooter.getShooterRPM());

    // PhotoEye
    SmartDashboard.putBoolean("HAS NOTE", intake.hasNote());
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