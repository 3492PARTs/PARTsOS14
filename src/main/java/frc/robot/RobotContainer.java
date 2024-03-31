// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.RunArmToLimitSwitchCmd;
import frc.robot.commands.Arm.Sequences.PivotArmCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroPivotEncodersCmdSeq;
import frc.robot.commands.Autos.AutoMoveForward;
import frc.robot.commands.Autos.AutoTurn;
import frc.robot.commands.Autos.StartAutoCmd;
import frc.robot.commands.Autos.AmpSide.AutoOneNoteAmpSide;
import frc.robot.commands.Autos.AmpSide.AutoTwoNoteAmpSide;
import frc.robot.commands.Autos.EmptySide.AutoOneNoteEmptySide;
import frc.robot.commands.Autos.EmptySide.AutoTwoNoteEmptySide;
import frc.robot.commands.Autos.Middle.AutoOneNoteMiddle;
import frc.robot.commands.Autos.Middle.AutoTwoNoteMiddle;
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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
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
  private final Climber climber = Climber.getInstance();
  private boolean climbMode = false;

  // Drive controls drivetrain, operator controls arm, intake, and shooter.
  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  public final Trigger zeroPivotTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger armGroundTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger noteTrigger = new Trigger(intake.hasNoteSupplier());

  // SmartDashboard chooser for auto tasks.
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    configureAutonomousCommands();
    configureSmartDashboardCommands();
    SmartDashboard.putData(arm);
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

    arm.setDefaultCommand(
        new RunCommand(() -> {
          // Manual control with a lower hard stop.
          if (Math.abs(operatorController.getRightY()) > .1 && !climbMode) {
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
              //if (Math.abs(arm.getRotationRate()) < 5)
              CommandScheduler.getInstance().schedule(new HoldArmInPositionCmd(arm.getAngle()));

            }
          }
        },
            arm));

    climber.setDefaultCommand(
        new RunCommand(() -> {
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
        },
            climber));

    zeroPivotTrigger.onTrue(new ZeroPivotEncodersCmdSeq());

    //* ----------------------------------------------------------------------------------- */
    //* Button Binding commands */
    //* ----------------------------------------------------------------------------------- */

    //* Change arm controls to climber controls */
    operatorController.povRight().onTrue(Commands.runOnce(() -> {
      climbMode = !climbMode;
      System.out.println("climb mode " + climbMode);
    }));

    //* ----------------------------------------------------------------------------------- */
    //* Shooting commands */
    //* ----------------------------------------------------------------------------------- */
    // Speaker
    operatorController.rightTrigger(.1)
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM),
            new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM)));

    //Amp
    //TODO: Test out setting bang bang to higher value
    operatorController.rightBumper()
        .onTrue(new ParallelRaceGroup(new BangBangShooterCmd(Constants.Shooter.AMP_RPM),
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

    // Run intake in
    operatorController.povLeft().whileTrue(new RunIntakeCmd(-1));

    //* ----------------------------------------------------------------------------------- */
    //* Arm commands */
    //* ----------------------------------------------------------------------------------- */
    if (!Constants.Arm.SYSID) {
      //*  Profiled Pivot Functions */
      // ground
      operatorController.x().onTrue(new PivotArmCmdSeq(Constants.Arm.GROUND));

      //speaker
      operatorController.y().onTrue(Commands.runOnce(() -> {
        //Schedule this outside of the command sequence so it stays running after the arm moves.
        CommandScheduler.getInstance().schedule(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM));
      }).andThen(new PivotArmCmdSeq(Constants.Arm.SPEAKER)));

      // home
      operatorController.b().onTrue(new PivotArmCmdSeq(Constants.Arm.HOME));

      // amp 
      operatorController.a().onTrue(new PivotArmCmdSeq(Constants.Arm.AMP));

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
    //Pre Match Dashboard
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.PRE_MATCH.tabName).add("Auto Mode", autoChooser)
        .withSize(2, 1) // make the widget 2x1
        .withPosition(0, 0); // place it in the top-left corner

    ShuffleboardLayout auto3NoteMiddleOptions = Dashboard
        .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.PRE_MATCH.tabName)
        .getLayout("3 Note Auto Options", BuiltInLayouts.kList)
        .withSize(2, 2).withPosition(2, 0);

    auto3NoteMiddleOptions.addBoolean("T: Go Amp, F: Go Empty", () -> {
      return false;
    }).withWidget(BuiltInWidgets.kToggleButton);
    auto3NoteMiddleOptions.addBoolean("Go Amp: T: Shoot Speaker, F: Shoot Amp", () -> {
      return false;
    }).withWidget(BuiltInWidgets.kToggleButton);

    // Autonomous Dashboard
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName)
        .addNumber("Gyro", driveTrain.getGyroAngleSupplier()).withWidget(BuiltInWidgets.kGyro);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).add(arm);
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addNumber("Arm Angle",
        arm.getAngleSupplier());
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addBoolean("Arm Limit",
        arm.getLimitSwitchSupplier());

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addBoolean("Note",
        intake.hasNoteSupplier());

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName).addNumber("Shooter RPM",
        shooter::getShooterRPM);

    // Teleoperated Dashboard
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).add(arm).withPosition(0, 0);
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addNumber("Arm Angle",
        arm.getAngleSupplier()).withPosition(2, 0);
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addBoolean("Arm Limit",
        arm.getLimitSwitchSupplier()).withPosition(4, 0);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addBoolean("Note",
        intake.hasNoteSupplier()).withPosition(0, 1);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName).addNumber("Shooter RPM",
        shooter::getShooterRPM).withPosition(0, 2);

    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.TELEOPERATED.tabName)
        .add(Camera.getInstance().getVideoSource()).withWidget(BuiltInWidgets.kCameraStream)
        .withSize(2, 1).withPosition(6, 0);
    // Debug Dashboard
  }

  public void configureSmartDashboardCommands() {
    if (Constants.Debug.debugMode) {
      // Zero Pivot Command
      SmartDashboard.putData("Zero Arm Sequence", new ZeroArmCmdSeq());
    }

    SmartDashboard.putBoolean("Auto Three Empty", false);

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

    SmartDashboard.putBoolean("Auto Three Empty Tester", SmartDashboard.getBoolean("Auto Three Empty", false));

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

    if (Constants.Debug.debugMode) {
      autoChooser.addOption("Test turn with color, red -> right, blue -> left", new StartAutoCmd());
    }
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

  public static boolean operatorButtonInterrupt() {
    return operatorController.leftBumper().getAsBoolean() ||
        operatorController.leftTrigger().getAsBoolean() ||
        operatorController.rightBumper().getAsBoolean() ||
        operatorController.rightTrigger().getAsBoolean() ||
        operatorController.a().getAsBoolean() ||
        operatorController.b().getAsBoolean() ||
        operatorController.x().getAsBoolean() ||
        operatorController.y().getAsBoolean();
  }

  public static boolean operatorJoystickInterrupt() {
    return Math.abs(RobotContainer.operatorController.getRightY()) > .1;
  }
}