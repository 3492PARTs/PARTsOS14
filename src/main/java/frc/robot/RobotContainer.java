// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.JoystickArmCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.RunArmToLimitSwitchCmd;
import frc.robot.commands.Arm.Sequences.PivotArmCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroPivotEncodersCmdSeq;
import frc.robot.commands.Autos.AutoMoveForward;
import frc.robot.commands.Autos.AutoTurn;
import frc.robot.commands.Autos.AmpSide.Start.StartAutoOneNoteAmpSide;
import frc.robot.commands.Autos.AmpSide.Start.StartAutoSpeakTwoNoteAmpSide;
import frc.robot.commands.Autos.EmptySide.Start.StartAutoOneNoteEmptySide;
import frc.robot.commands.Autos.EmptySide.Start.StartAutoTwoNoteEmptySide;
import frc.robot.commands.Autos.Middle.AutoOneNoteMiddle;
import frc.robot.commands.Autos.Middle.AutoTwoNoteMiddle;
import frc.robot.commands.Autos.Middle.Start.StartAutoFourNoteMiddle;
import frc.robot.commands.Autos.Middle.Start.StartAutoThreeNoteMiddle;
import frc.robot.commands.Intake.RunIntakeCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.Sequences.IntakeArmToPositionCmdSeq;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.commands.Shooter.ShootCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Candle.Color;
import frc.robot.util.Dashboard;
import frc.robot.util.Logger;
import frc.robot.subsystems.Climber;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
  private final Climber climber = Climber.getInstance();
  private boolean climbMode = false;

  // Drive controls drivetrain, operator controls arm, intake, and shooter.
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final Trigger zeroPivotTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger armGroundTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger noteTrigger = new Trigger(intake.hasNoteSupplier());

  // SmartDashboard chooser for auto tasks.
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private GenericEntry ampOrEmpty;
  private GenericEntry speakerOrAmp;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    //configureSmartDashboardCommands();
    configureDashboard();
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
    //* ----------------------------------------------------------------------------------- */
    //* Default commands */
    //* ----------------------------------------------------------------------------------- */
    driveTrain.setDefaultCommand(
        new RunCommand(() -> {
          driveTrain.driveArcade(-driveController.getLeftY(), driveController.getRightX());
        },
            driveTrain));

    // Default is hold arm in place
    arm.setDefaultCommand(
        new RunCommand(() -> {
          arm.setSpeed(0);
          if (arm.getAngle() > 2) {
            //if (Math.abs(arm.getRotationRate()) < 5)
            CommandScheduler.getInstance().schedule(
                new HoldArmInPositionCmd(arm.getAngle()).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

          }
        }, arm));

    climber.setDefaultCommand(new RunCommand(() -> {
      if (climbMode) {
        if (Math.abs(operatorController.getLeftY()) > .1) {
          climber.setLeftSpeed(operatorController.getLeftY());
        } else {
          climber.setLeftSpeed(0);
        }
        if (Math.abs(operatorController.getRightY()) > .1) {
          climber.setRightSpeed(operatorController.getRightY());
        } else {
          climber.setRightSpeed(0);
        }
      } else {
        climber.setLeftSpeed(0);
        climber.setRightSpeed(0);
      }
    }, climber));

    zeroPivotTrigger.onTrue(new ZeroPivotEncodersCmdSeq());

    //* ----------------------------------------------------------------------------------- */
    //* Button Binding commands */
    //* ----------------------------------------------------------------------------------- */

    //* Change arm controls to climber controls */
    operatorController.povRight().onTrue(Commands.runOnce(() -> {
      CommandScheduler.getInstance().cancelAll();
      climbMode = !climbMode;
    }).andThen(new ProfiledPivotArmCmd(Constants.Arm.GROUND)));

    //* ----------------------------------------------------------------------------------- */
    //* Shooting commands */
    //* ----------------------------------------------------------------------------------- */
    // Speaker
    operatorController.rightTrigger(.1)
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM)));

    //Amp
    //TODO: Test out setting bang bang to higher value
    operatorController.rightBumper().onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.AMP_RPM),
        new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.AMP_RPM)));

    //* ----------------------------------------------------------------------------------- */
    //* Intake commands */
    //* ----------------------------------------------------------------------------------- */
    // Run intake in until note detected, send to home after. 
    operatorController.leftTrigger(.1)
        .onTrue(new IntakeArmToPositionCmdSeq(Constants.Intake.INTAKE_SPEED, Constants.Arm.HOME));

    // Run intake out
    operatorController.leftBumper().whileTrue(new RunIntakeCmd(1));

    //* ----------------------------------------------------------------------------------- */
    //* Manual commands */
    //* ----------------------------------------------------------------------------------- */
    // Shoot out full speed
    operatorController.povUp().whileTrue(new ShootCmd(1));

    //Lobbing Shooting
    operatorController.povDown()
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(700), new RunIntakeWhenShooterAtRPMCmd(700)));

    // Run intake in
    operatorController.povLeft().whileTrue(new RunIntakeCmd(-1));

    //* ----------------------------------------------------------------------------------- */
    //* Arm commands */
    //* ----------------------------------------------------------------------------------- */
    //Trigger for manual control to run arm manually
    new Trigger(() -> {
      return Math.abs(operatorController.getRightY()) > Constants.Arm.JOYSTICK_CONTROL_LIMIT && !climbMode;
    }).onTrue(new JoystickArmCmd(operatorController));

    if (!Constants.Arm.SYSID) {
      //*  Profiled Pivot Functions */
      // ground
      operatorController.x()
          .onTrue(new PivotArmCmdSeq(Constants.Arm.GROUND).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

      //speaker
      operatorController.y().onTrue(Commands.runOnce(() -> {
        //Schedule this outside of the command sequence so it stays running after the arm moves.
        CommandScheduler.getInstance().schedule(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
      }).andThen(new PivotArmCmdSeq(Constants.Arm.SPEAKER).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

      // home
      operatorController.b()
          .onTrue(new PivotArmCmdSeq(Constants.Arm.HOME).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

      // amp 
      operatorController.a()
          .onTrue(new PivotArmCmdSeq(Constants.Arm.AMP).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    } else {
      //* SysID Functions */
      operatorController.a().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
      operatorController.b().whileTrue(arm.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
      operatorController.x().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
      operatorController.y().whileTrue(arm.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    //* ----------------------------------------------------------------------------------- */
    //* Debug commands */
    //* ----------------------------------------------------------------------------------- */
    if (Constants.Debug.debugMode) {
      operatorController.povDown().onTrue(new PivotArmCmdSeq(37));
      driveController.x().onTrue(new RunArmToLimitSwitchCmd());
    }
  }

  public void configureCandleBindings() {
    //* Note sensor */
    // Has a note turn green
    noteTrigger.onTrue(Commands.runOnce(() -> {
      Logger.getInstance().logBoolean("Note Trigger", true);
      candle.setColor(Color.GREEN);
    }, candle));

    // No note turn blue
    noteTrigger.onFalse(Commands.runOnce(() -> {
      Logger.getInstance().logBoolean("Note Trigger", false);
      candle.setColor(Color.BLUE);
    }, candle));

    //* Arm Limit Switch */
    // Arm on ground, turn orange
    armGroundTrigger.onTrue(Commands.runOnce(() -> {
      Logger.getInstance().logBoolean("Arm Trigger", true);
      candle.setColor(Color.ORANGE);
    }, candle));

    // Arm not on ground, turn default blue,
    // unless we have a note, so it stays the note triggered color
    armGroundTrigger.onFalse(Commands.runOnce(() -> {
      Logger.getInstance().logBoolean("Arm Trigger", false);
    }).andThen(
        Commands.runOnce(() -> {
          candle.setColor(Color.BLUE);
        }, candle).onlyIf(intake.doesNotHaveNoteSupplier())));
  }

  public void initializeCandleState() {
    if (intake.hasNote())
      candle.setColor(Color.GREEN);
    else if (arm.getLimitSwitch())
      candle.setColor(Color.ORANGE);
    else
      candle.setColor(Color.BLUE);
  }

  public void removeBindings() {
    arm.removeDefaultCommand();
    driveTrain.removeDefaultCommand();
    //zeroPivotTrigger.onTrue(new NullCmd()); //idk if this is causing pause in autos or not?
  }

  public void configureDashboard() {
    //* ----------------------------------------------------------------------------------- */
    //* Pre Match Dashboard */
    //* ----------------------------------------------------------------------------------- */
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.PRE_MATCH.tabName).add("Auto Mode", autoChooser)
        .withSize(2, 1) // make the widget 2x1
        .withPosition(0, 0); // place it in the top-left corner

    ShuffleboardLayout auto3NoteMiddleOptions = Dashboard
        .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.PRE_MATCH.tabName)
        .getLayout("3 Note Auto Options", BuiltInLayouts.kList)
        .withSize(2, 2).withPosition(2, 0);

    ampOrEmpty = auto3NoteMiddleOptions.add("T: Go Empty, F: Go Amp", false).withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

    speakerOrAmp = auto3NoteMiddleOptions.add("Amp Chosen: T: Shoot Speaker, F: Shoot Amp", false)
        .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    //* ----------------------------------------------------------------------------------- */
    //* Autonomous Dashboard */
    //* ----------------------------------------------------------------------------------- */
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName)
        .addNumber("Gyro", driveTrain.getGyroAngleSupplier()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 0);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).add(arm).withPosition(2, 0);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addBoolean("Arm Limit",
        arm.getLimitSwitchSupplier()).withPosition(2, 1);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addNumber("Arm Angle",
        arm.getAngleSupplier()).withPosition(3, 1);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addBoolean("Note",
        intake.hasNoteSupplier()).withPosition(2, 2);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addNumber("Shooter RPM",
        shooter::getShooterRPM).withPosition(3, 2);

    //* ----------------------------------------------------------------------------------- */
    //* Teleoperated Dashboard */
    //* ----------------------------------------------------------------------------------- */
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addBoolean("Arm Limit",
        arm.getLimitSwitchSupplier()).withPosition(0, 0);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addNumber("Arm Angle",
        arm.getAngleSupplier()).withPosition(1, 0);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName)
        .add("Zero Arm Sequence", new ZeroArmCmdSeq()).withPosition(0, 3);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addBoolean("Climber Control",
        () -> {
          return climbMode;
        }).withPosition(2, 0);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addBoolean("Note",
        intake.hasNoteSupplier()).withPosition(0, 1);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addNumber("Shooter RPM",
        shooter::getShooterRPM).withPosition(1, 1);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName)
        .add(Camera.getInstance().getVideoSource()).withWidget(BuiltInWidgets.kCameraStream)
        .withSize(7, 6).withPosition(3, 0);

    //* ----------------------------------------------------------------------------------- */
    //* Debug Dashboard */
    //* ----------------------------------------------------------------------------------- */
    if (Constants.Debug.debugMode) {
      ShuffleboardLayout driveTrainLayout = Dashboard
          .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.DEBUG.tabName)
          .getLayout("Drive Train", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0)
          .withProperties(Map.of("Label position", "TOP"));

      driveTrainLayout.add(driveTrain);
      driveTrainLayout.addNumber("Gyro", driveTrain.getGyroAngleSupplier());
      driveTrainLayout.addDouble("Left Drive Distance", driveTrain::leftDistance);
      driveTrainLayout.addDouble("Right Drive Distance", driveTrain::rightDistance);

      ShuffleboardLayout armLayout = Dashboard
          .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.DEBUG.tabName)
          .getLayout("Arm", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0)
          .withProperties(Map.of("Label position", "TOP"));

      armLayout.add(arm);

      armLayout.add("Zero Arm Sequence", new ZeroArmCmdSeq()).withProperties(Map.of("Label position", "HIDDEN"));

      armLayout.addBoolean("Arm Limit",
          arm.getLimitSwitchSupplier());

      armLayout.addNumber("Arm Angle",
          arm.getAngleSupplier());

      armLayout.addNumber("Arm Voltage",
          arm::getAverageVoltage);

      ShuffleboardLayout intakeLayout = Dashboard
          .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.DEBUG.tabName)
          .getLayout("Intake", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0)
          .withProperties(Map.of("Label position", "TOP"));

      intakeLayout.add(intake);

      intakeLayout.addBoolean("Has Note",
          intake.hasNoteSupplier());

      ShuffleboardLayout shooterLayout = Dashboard
          .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.DEBUG.tabName)
          .getLayout("Shooter", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0)
          .withProperties(Map.of("Label position", "TOP"));

      shooterLayout.add(shooter);

      shooterLayout.addDouble("Shooter RPM",
          shooter::getShooterRPM);

      shooterLayout.addDouble("Left Motor Velocity",
          shooter::getLeftVelocity);

      shooterLayout.addDouble("Right Motor Velocity",
          shooter::getRightVelocity);

      ShuffleboardLayout climberLayout = Dashboard
          .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.DEBUG.tabName)
          .getLayout("Climber", BuiltInLayouts.kList).withSize(2, 4).withPosition(8, 0)
          .withProperties(Map.of("Label position", "TOP"));

      climberLayout.add(climber);

      climberLayout.addBoolean("Climber Control",
          () -> {
            return climbMode;
          });
    }

  }

  /*
  public void configureSmartDashboardCommands() {
    if (Constants.Debug.debugMode) {
      // Zero Pivot Command
      SmartDashboard.putData("Zero Arm Sequence", new ZeroArmCmdSeq());
    }
  }
  
  public void updateSmartDashboard() {
    // Shooter
    SmartDashboard.putNumber("Shooter RPM", shooter.getShooterRPM());
    SmartDashboard.putNumber("Shooter Left Velocity", shooter.getLeftVelocity());
    SmartDashboard.putNumber("Shooter Right Velocity", shooter.getRightVelocity());
  
    // PhotoEye
    SmartDashboard.putBoolean("HAS NOTE", intake.hasNote());
  
    // LimitSwitch
    SmartDashboard.putBoolean("Arm Switch", arm.getLimitSwitch());
  
    // Arm Angle
    SmartDashboard.putNumber("Arm Angle", arm.getAngle());
  
    // Arm Angle
    SmartDashboard.putNumber("Arm Voltage", arm.getAverageVoltage());
  
    // Climber or Arm Controls
    SmartDashboard.putBoolean("Climber control", climbMode);
  
    if (Constants.Debug.debugMode) {
      // Drive
      SmartDashboard.putNumber("Left Drive Distance", driveTrain.leftDistance());
      SmartDashboard.putNumber("Right Drive Distance", driveTrain.rightDistance());
  
      SmartDashboard.putNumber("Gyro Angle", driveTrain.getGyroAngle());
    }
  }
  */

  public void configureAutonomousCommands() {
    //SmartDashboard.putData("choose auto mode", autoChooser);

    autoChooser.addOption("Move Forward", new AutoMoveForward());
    autoChooser.addOption("Move Turn", new AutoTurn());

    autoChooser.addOption("One Note Middle", new AutoOneNoteMiddle());
    autoChooser.addOption("Two Note Middle", new AutoTwoNoteMiddle());

    autoChooser.addOption("Three Note Middle", new StartAutoThreeNoteMiddle(ampOrEmpty, speakerOrAmp));
    autoChooser.addOption("Four Note Middle", new StartAutoFourNoteMiddle());

    autoChooser.addOption("One Note Amp Side ", new StartAutoOneNoteAmpSide());
    autoChooser.addOption("Two Note Amp Side", new StartAutoSpeakTwoNoteAmpSide());
    autoChooser.addOption("Two Note Speak Amp Side", new StartAutoSpeakTwoNoteAmpSide());

    autoChooser.addOption("One Note Empty Side", new StartAutoOneNoteEmptySide());
    autoChooser.addOption("Two Note Empty Side", new StartAutoTwoNoteEmptySide());

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