// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.RunArmToLimitSwitchCmd;
import frc.robot.commands.Arm.ZeroPivotEncodersCmd;
import frc.robot.commands.Arm.Sequences.PivotArmCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Autos.AutoMoveForward;
import frc.robot.commands.Autos.AutoTurn;
import frc.robot.commands.Autos.AmpSide.AutoOneNoteAmpSide;
import frc.robot.commands.Autos.AmpSide.AutoTwoNoteAmpSide;
import frc.robot.commands.Autos.EmptySide.AutoOneNoteEmptySide;
import frc.robot.commands.Autos.EmptySide.AutoTwoNoteEmptySide;
import frc.robot.commands.Autos.Middle.AutoOneNoteMiddle;
import frc.robot.commands.Autos.Middle.AutoTwoNoteMiddle;
import frc.robot.commands.Intake.IntakePhotoEyeArmPosCmd;
import frc.robot.commands.Intake.RunIntakeCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.Sequences.IntakeArmPositionCmdSeq;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.commands.Shooter.ShootCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Candle.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

  public final Trigger zeroPivotTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger armGroundTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger noteTrigger = new Trigger(() -> {
    return intake.hasNote();
  });

  // SmartDashboard chooser for auto tasks.
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    configureAutonomousCommands();
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

    //TODO: I found a way to remove default below test it
    zeroPivotTrigger.onTrue(Commands.waitSeconds(.2).andThen(new ZeroPivotEncodersCmd()));

    //TODO: A much better arm default command since calling schedule is bad. 
    /*
    arm.setDefaultCommand(new ConditionalCommand(new RunCommand(() -> {
      //at bottom limit
      if (arm.getSwitch()) {
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
    }, arm),
        Commands.runOnce(() -> {
          arm.setPivotSpeed(0);
        }).andThen(new HoldArmInPositionCmd(arm.getAngle()).onlyIf(() -> { // IDK If this will set multiple times or not.
          return arm.getAngle() > 2;
        })),
        () -> {
          return Math.abs(operatorController.getRightY()) > .1;
        }));
    */

    arm.setDefaultCommand(
        new RunCommand(() -> {
          // Manual control with a lower hard stop.
          if (Math.abs(operatorController.getRightY()) > .1) {
            //at bottom limit
            if (arm.getLimitSwitch()) {
              // Negative controller value is up on arm
              if (operatorController.getRightY() < 0)
                arm.setSpeed(operatorController.getRightY());
              else
                arm.setSpeed(0);
            }
            // at top limit
            else if (arm.getAngle() >= Constants.Arm.UPPER_BOUND) {
              // Positive controller value is down on arm
              if (operatorController.getRightY() > 0)
                arm.setSpeed(operatorController.getRightY());
              else
                arm.setSpeed(0);
            }
            // full manual, in safe bounds
            else
              arm.setSpeed(operatorController.getRightY());
          }
          // hold arm in current position
          else {
            arm.setSpeed(0);
            if (arm.getAngle() > 2) {
              //TODO: Verify no issues. but i believe this is a better way to schedule commands.
              //if (Math.abs(arm.getRotationRate()) < 5)
              CommandScheduler.getInstance().schedule(new HoldArmInPositionCmd(arm.getAngle()));

            }
          }
        },
            arm));

    // Shooting Functions
    // Speaker
    operatorController.rightTrigger(.1)
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM)));

    //Amp
    operatorController.rightBumper()
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.AMP_RPM),
            new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.AMP_RPM)));

    // Intake Functions
    // Run intake until note detected, send to home after. 
    operatorController.leftTrigger(.1)
        .onTrue(new IntakePhotoEyeArmPosCmd(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME));

    //TODO: Verify this is a better way. operatorController.leftTrigger(.1).onTrue(new IntakeArmPositionCmdSeq(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME));

    // Run intake in
    operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));

    // Testing
    if (Constants.TESTING) {
      operatorController.povDown().onTrue(new PivotArmCmdSeq(45));
      driveController.x().onTrue(new RunArmToLimitSwitchCmd());
    }

    // Manual Functions
    // Shoot out full speed
    operatorController.povUp().whileTrue(new ShootCmd(1));

    // Run intake out
    operatorController.povLeft().whileTrue(new RunIntakeCmd(-1));

    if (!Constants.Arm.SYSID) {
      // Profiled Pivot Functions
      // ground
      operatorController.x().onTrue(new PivotArmCmdSeq(Constants.Arm.GROUND));

      //speaker
      operatorController.y().onTrue(Commands.runOnce(() -> {
        //TODO: Verify no issues calling new way new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM).schedule();
        CommandScheduler.getInstance().schedule(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM));
      }).andThen(new PivotArmCmdSeq(Constants.Arm.SPEAKER)));

      // home
      operatorController.b().onTrue(new PivotArmCmdSeq(Constants.Arm.HOME));

      // amp 
      operatorController.a().onTrue(new PivotArmCmdSeq(Constants.Arm.AMP));
    }
    // SysID
    else {
      operatorController.a().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
      operatorController.b().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
      operatorController.x().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
      operatorController.y().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  public void configureCandleBindings() {
    candle.setColor(Color.BLUE);

    // Note sensor
    noteTrigger.onTrue(Commands.runOnce(() -> {
      candle.setColor(Color.GREEN);
    }, candle));

    noteTrigger.onFalse(Commands.runOnce(() -> {
      candle.setColor(Color.BLUE);
    }, candle));

    // Arm Limit Switch
    armGroundTrigger.onTrue(Commands.runOnce(() -> {
      candle.setColor(Color.ORANGE);
    }, candle));

    armGroundTrigger.onFalse(Commands.runOnce(() -> {
      candle.setColor(Color.BLUE);
    }, candle).onlyIf(() -> {
      return !intake.hasNote();
    }));
  }

  public void removeBindings() {
    arm.removeDefaultCommand();
    driveTrain.removeDefaultCommand();
    zeroPivotTrigger.onTrue(null);
  }

  public void updateSmartDashboard() {
    // Shooter
    SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());

    // PhotoEye
    SmartDashboard.putBoolean("HAS NOTE", intake.hasNote());

    // LimitSwitch
    SmartDashboard.putBoolean("Arm Switch", arm.getLimitSwitch());

    // Arm Angle
    SmartDashboard.putNumber("Arm Angle", arm.getAngle());

    if (Constants.TESTING) {
      // Drive
      SmartDashboard.putNumber("Left Drive Distance", driveTrain.leftDistance());
      SmartDashboard.putNumber("Right Drive Distance", driveTrain.rightDistance());

      SmartDashboard.putNumber("Gyro Angle", driveTrain.getGyroAngle());

      // Pivot
      SmartDashboard.putData("Zero Arm Sequence", new ZeroArmCmdSeq());
    }

  }

  public void configureAutonomousCommands() {
    SmartDashboard.putData("choose auto mode", autoChooser);

    // SIDE INDEPENDENT AUTOS
    autoChooser.addOption("Move Forward", new AutoMoveForward());
    autoChooser.addOption("Move Turn", new AutoTurn());
    autoChooser.addOption("One Note Middle", new AutoOneNoteMiddle());
    autoChooser.addOption("Two Note Middle", new AutoTwoNoteMiddle());

    //RED AUTOS
    autoChooser.addOption("RED: One Note Amp Side ", new AutoOneNoteAmpSide(1));
    autoChooser.addOption("RED: One Note Empty Side", new AutoOneNoteEmptySide(1));
    autoChooser.addOption("RED: Two Note Amp Side", new AutoTwoNoteAmpSide(1));
    autoChooser.addOption("RED: Two Note Empty Side", new AutoTwoNoteEmptySide(1));

    //BLUE AUTOS
    autoChooser.addOption("BLUE: One Note Amp Side ", new AutoOneNoteAmpSide(-1));
    autoChooser.addOption("BLUE: One Note Empty Side", new AutoOneNoteEmptySide(-1));
    autoChooser.addOption("BLUE: Two Note Amp Side", new AutoTwoNoteAmpSide(-1));
    autoChooser.addOption("BLUE: Two Note Empty Side", new AutoTwoNoteEmptySide(-1));
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