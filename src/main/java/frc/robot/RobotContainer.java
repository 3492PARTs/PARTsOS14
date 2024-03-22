// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.RunArmToZeroCmd;
import frc.robot.commands.Arm.ZeroPivotEncodersCmd;
import frc.robot.commands.Autos.AutoMoveForward;
import frc.robot.commands.Autos.AutoOneNoteEmptySide;
import frc.robot.commands.Autos.AutoOneNoteMiddlePos;
import frc.robot.commands.Autos.AutoTurn;
import frc.robot.commands.Autos.AutoOneNoteAmpSidePos;
import frc.robot.commands.Autos.AutoTwoNoteEmptySpacePos;
import frc.robot.commands.Autos.AutoTwoNoteMiddlePos;
import frc.robot.commands.Autos.AutoTwoNoteAmpSidePos;
import frc.robot.commands.IntakeShoot.RunIntakeCmd;
import frc.robot.commands.IntakeShoot.RunIntakePhotoEyeTeleopCmd;
import frc.robot.commands.IntakeShoot.ShootCmd;
import frc.robot.commands.IntakeShoot.ShootInAmpCmd;
import frc.robot.commands.IntakeShoot.ShootInSpeakerCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public final Trigger zeroPivotTrigger = new Trigger(arm.getSwitchSupplier());

  // SmartDashboard chooser for auto tasks.
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the trigger bindings
    // configureBindings();

    SmartDashboard.putData("choose auto mode", autoChooser);
    // SIDE INDEPENDENT AUTOS
    autoChooser.addOption("Move Forward", new AutoMoveForward());
    autoChooser.addOption("Move Turn", new AutoTurn());
    autoChooser.addOption("One Note Middle", new AutoOneNoteMiddlePos());
    autoChooser.addOption("Two Note Middle", new AutoTwoNoteMiddlePos());

    //RED AUTOS
    autoChooser.addOption("RED: One Note Amp Side ", new AutoOneNoteAmpSidePos(1));
    autoChooser.addOption("RED: One Note Empty Side", new AutoOneNoteEmptySide(1));
    autoChooser.addOption("RED: Two Note Amp Side", new AutoTwoNoteAmpSidePos(1));
    autoChooser.addOption("RED: Two Note Empty Side", new AutoTwoNoteEmptySpacePos(1));

    //BLUE AUTOS
    autoChooser.addOption("BLUE: One Note Amp Side ", new AutoOneNoteAmpSidePos(-1));
    autoChooser.addOption("BLUE: One Note Empty Side", new AutoOneNoteEmptySide(-1));
    autoChooser.addOption("BLUE: Two Note Amp Side", new AutoTwoNoteAmpSidePos(-1));
    autoChooser.addOption("BLUE: Two Note Empty Side", new AutoTwoNoteEmptySpacePos(-1));

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
    driveController.x()
        .onTrue(new RunArmToZeroCmd());

    zeroPivotTrigger.onTrue(Commands.waitSeconds(.2).andThen(new ZeroPivotEncodersCmd()));

    arm.setDefaultCommand(
        new RunCommand(() -> {
          // Manual control with a lower hard stop.
          if (Math.abs(operatorController.getRightY()) > .1) {

            //at bottom limit
            if (Arm.getInstance().getSwitch()) {
              if (operatorController.getRightY() < 0)
                arm.setPivotSpeed(operatorController.getRightY());
              else
                arm.setPivotSpeed(0);
            }
            // at top limit
            else if (arm.getAngle() >= Constants.Arm.UPPER_BOUND) {
              if (operatorController.getRightY() > 0)
                arm.setPivotSpeed(operatorController.getRightY());
              else
                arm.setPivotSpeed(0);
            }
            // full manual, in safe bounds
            else
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
    if (!Constants.Arm.SYSID) {
      operatorController.x().onTrue(new ProfiledPivotArmCmd(Constants.Arm.GROUND)); // ground
      operatorController.y().onTrue(new ProfiledPivotArmCmd(Constants.Arm.SPEAKER)); // speaker
      operatorController.b().onTrue(new ProfiledPivotArmCmd(Constants.Arm.HOME)); // home
      operatorController.a().onTrue(new ProfiledPivotArmCmd(Constants.Arm.AMP)); // amp
    }

    operatorController.povRight().onTrue(new ArmToPositionTeleopCmd(-2)); //do not use

    operatorController.rightTrigger(.1).whileTrue(new ShootInSpeakerCmd());
    operatorController.rightBumper().onTrue(new ShootInAmpCmd());

    operatorController.leftTrigger(.1)
        .onTrue(new RunIntakePhotoEyeTeleopCmd(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME));
    operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));

    operatorController.povUp().whileTrue(new ShootCmd());

    // Profiled Pivot Buttons

    //operatorController.x().onTrue(new ProfiledPivotArmCmd(120));
    // ground
    //operatorController.y().onTrue(new ProfiledPivotArmCmd(42.8, 3.0, 0.0, 0.0));
    // speaker
    //operatorController.b().onTrue(new ProfiledPivotArmCmd(30, 2.7, 0.0, 0.0)); //
    // home
    // operatorController.a().onTrue(new ProfiledPivotArmCmd(-5.09, 3.0, 0.3, 0.0));

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

    SmartDashboard.putNumber("gyro angle", driveTrain.getGyroAngle());
    SmartDashboard.putNumber("graph angle", driveTrain.getGyroAngle());
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

    SmartDashboard.putNumber("Pos", Arm.getInstance().getAlternateEncoderPosition());

    // Shooter
    SmartDashboard.putNumber("shooter RPM", shooter.getShooterRPM());

    // PhotoEye
    SmartDashboard.putBoolean("HAS NOTE", intake.hasNote());

    SmartDashboard.putBoolean("Arm Switch", Arm.getInstance().getSwitch());
    //SmartDashboard.putBoolean("Arm Switch Buffer", armLimitBuffer);
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