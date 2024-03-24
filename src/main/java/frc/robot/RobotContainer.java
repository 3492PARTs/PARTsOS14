// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmToPositionTeleopCmd;
import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.RunArmToLimitSwitchCmd;
import frc.robot.commands.Arm.ZeroPivotEncodersCmd;
import frc.robot.commands.Arm.Sequences.PivotArmCmdSeq;
import frc.robot.commands.Autos.AutoMoveForward;
import frc.robot.commands.Autos.AutoOneNoteEmptySide;
import frc.robot.commands.Autos.AutoOneNoteMiddlePos;
import frc.robot.commands.Autos.AutoTurn;
import frc.robot.commands.Autos.AutoOneNoteAmpSidePos;
import frc.robot.commands.Autos.AutoTwoNoteEmptySpacePos;
import frc.robot.commands.Autos.AutoTwoNoteMiddlePos;
import frc.robot.commands.Intake.IntakePhotoEyeArmPosCmd;
import frc.robot.commands.Intake.RunIntakeCmd;
import frc.robot.commands.Intake.RunIntakeWhenAtRPMCmd;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.commands.Shooter.ShootCmd;
import frc.robot.commands.Shooter.ShootInAmpCmd;
import frc.robot.commands.Autos.AutoTwoNoteAmpSidePos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Candle.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
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
  private final Candle candle = Candle.getInstance();

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

    //candle.runRainbowAnimationCommand();
    candle.setColor(Color.BLUE);
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

    //zeroPivotTrigger.onTrue(Commands.waitSeconds(.2).andThen(new ZeroPivotEncodersCmd()));

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
            arm.setPivotSpeed(0);
            if (arm.getAngle() > 2) {
              new HoldArmInPositionCmd(arm.getAngle()).schedule();
            }
          }
        },
            arm));

    // Shooting Functions
    operatorController.rightTrigger(.1)
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeWhenAtRPMCmd(Constants.Shooter.SPEAKER_RPM)));

    operatorController.rightBumper()
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.AMP_RPM),
            new RunIntakeWhenAtRPMCmd(Constants.Shooter.AMP_RPM)));

    // Intake Functions
    operatorController.leftTrigger(.1)
        .onTrue(new IntakePhotoEyeArmPosCmd(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME));

    operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));

    // Testing
    if (Constants.TESTING) {
      operatorController.povDown().onTrue(new PivotArmCmdSeq(45));
      driveController.x().onTrue(new RunArmToLimitSwitchCmd());
    }

    // Manual Functions
    operatorController.povUp().whileTrue(new ShootCmd());
    operatorController.povLeft().whileTrue(new RunIntakeCmd(-1));

    if (!Constants.Arm.SYSID) {
      // Profiled Pivot Functions
      operatorController.x().onTrue(new PivotArmCmdSeq(Constants.Arm.GROUND)); // ground
      operatorController.y().onTrue(Commands.runOnce(() -> {
        new PivotArmCmdSeq(Constants.Shooter.WARMUP_SPEAKER_RPM).schedule();
      }).andThen(new PivotArmCmdSeq(Constants.Arm.SPEAKER))); //speaker
      operatorController.b().onTrue(new PivotArmCmdSeq(Constants.Arm.HOME)); // home
      operatorController.a().onTrue(new PivotArmCmdSeq(Constants.Arm.AMP)); // amp
    }
    // SysID
    else {
      operatorController.a().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
      operatorController.b().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
      operatorController.x().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
      operatorController.y().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  public void removeBindings() {
    arm.removeDefaultCommand();
    driveTrain.removeDefaultCommand();
  }

  public void displaySmartDashboard() {
    // Drive
    SmartDashboard.putNumber("left Drive Distance", driveTrain.leftDistance());
    SmartDashboard.putNumber("right Drive Distance", driveTrain.rightDistance());

    SmartDashboard.putNumber("gyro angle", driveTrain.getGyroAngle());
    SmartDashboard.putNumber("graph angle", driveTrain.getGyroAngle());

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

    // LimitSwitch
    SmartDashboard.putBoolean("Arm Switch", Arm.getInstance().getSwitch());
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

  public static boolean operatorInterrupt() {
    return operatorController.leftBumper().getAsBoolean() ||
        operatorController.leftTrigger().getAsBoolean() ||
        operatorController.a().getAsBoolean() ||
        operatorController.b().getAsBoolean() ||
        operatorController.x().getAsBoolean() ||
        operatorController.y().getAsBoolean();
  }
}