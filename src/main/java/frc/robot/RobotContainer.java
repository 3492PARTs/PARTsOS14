// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.HoldArmInPositionCmd;
import frc.robot.commands.Arm.ProfiledPivotArmCmd;
import frc.robot.commands.Arm.RunArmToLimitSwitchCmd;
import frc.robot.commands.Arm.Sequences.PivotArmCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroArmCmdSeq;
import frc.robot.commands.Arm.Sequences.ZeroPivotEncodersCmdSeq;
import frc.robot.commands.Autos.Middle.AutoOneNoteMiddle;
import frc.robot.commands.Intake.RunIntakeCmd;
import frc.robot.commands.Intake.RunIntakeWhenShooterAtRPMCmd;
import frc.robot.commands.Intake.Sequences.IntakeArmToPositionCmdSeq;
import frc.robot.commands.Intake.Sequences.IntakeCmdSeq;
import frc.robot.commands.Shooter.BangBangShooterCmd;
import frc.robot.commands.Shooter.ShootCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Candle.Color;
import frc.robot.util.CommandSwerveDrivetrain;
import frc.robot.util.Dashboard;
import frc.robot.util.Logger;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final Arm arm = Arm.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Candle candle = Candle.getInstance();
  private boolean climbMode = false;

  // Drive controls drivetrain, operator controls arm, intake, and shooter.
  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  public final Trigger zeroPivotTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger armGroundTrigger = new Trigger(arm.getLimitSwitchSupplier());
  public final Trigger noteTrigger = new Trigger(intake.hasNoteSupplier());

  // SmartDashboard chooser for auto tasks.
  private SendableChooser<Command> autoChooser;
  //private GenericEntry ampOrEmpty;
  //private GenericEntry speakerOrAmp;

  // Swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    //configureSmartDashboardCommands();
    
    configureAutonomousCommands();
    configureDashboard();
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
    /*
    driveTrain.setDefaultCommand(
        new RunCommand(() -> {
          driveTrain.driveArcade(-driveController.getLeftY(), driveController.getRightX());
        },
            driveTrain));
    */

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> 
          drive.withVelocityX(-MathUtil.applyDeadband(driveController.getLeftY(), .1) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-MathUtil.applyDeadband(driveController.getLeftX(), .1) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-MathUtil.applyDeadband(driveController.getRightX(), .1) * MaxAngularRate)// Drive counterclockwise with negative X (left)    
    ));

    driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));

    driveController.povRight().onTrue(drivetrain.SnapToAngle(90));
    // reset the field-centric heading on left bumper press
    driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

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


    zeroPivotTrigger.onTrue(new ZeroPivotEncodersCmdSeq());

    //* ----------------------------------------------------------------------------------- */
    //* Button Binding commands */
    //* ----------------------------------------------------------------------------------- */

    //* Change arm controls to climber controls */
    operatorController.povRight().onTrue(Commands.runOnce(() -> {
      climbMode = !climbMode;
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

    //Lobbing Shooting
    operatorController.povDown().onTrue(new ParallelRaceGroup(new BangBangShooterCmd(700),
        new RunIntakeWhenShooterAtRPMCmd(700)));

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
        CommandScheduler.getInstance().schedule(new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
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
    //zeroPivotTrigger.onTrue(new NullCmd()); //idk if this is causing pause in autos or not?
  }

  public void configureDashboard() {
    //* ----------------------------------------------------------------------------------- */
    //* Pre Match Dashboard */
    //* ----------------------------------------------------------------------------------- */
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.PRE_MATCH.tabName).add("Auto Mode", autoChooser)
        .withSize(2, 1) // make the widget 2x1
        .withPosition(0, 0); // place it in the top-left corner

    /* 
    ShuffleboardLayout auto3NoteMiddleOptions = Dashboard
        .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.PRE_MATCH.tabName)
        .getLayout("3 Note Auto Options", BuiltInLayouts.kList)
        .withSize(2, 2).withPosition(2, 0);

    ampOrEmpty = auto3NoteMiddleOptions.add("T: Go Empty, F: Go Amp", false).withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

    speakerOrAmp = auto3NoteMiddleOptions.add("Amp Chosen: T: Shoot Speaker, F: Shoot Amp", false)
        .withWidget(BuiltInWidgets.kToggleButton).getEntry();
    */

    //* ----------------------------------------------------------------------------------- */
    //* Autonomous Dashboard */
    //* ----------------------------------------------------------------------------------- */
    Dashboard.getDashboardTab(frc.robot.Constants.Dashboard.Tabs.AUTONOMOUS.tabName)
        .addNumber("Gyro", () -> 0).withWidget(BuiltInWidgets.kGyro).withPosition(0, 0);

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

      //driveTrainLayout.add(driveTrain);
      //driveTrainLayout.addNumber("Gyro", driveTrain.getGyroAngleSupplier());
      //driveTrainLayout.addDouble("Left Drive Distance", driveTrain::leftDistance);
      //driveTrainLayout.addDouble("Right Drive Distance", driveTrain::rightDistance);

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

      /*shooterLayout.addDouble("Right Motor Velocity",
          shooter::getRightVelocity);
*/
      ShuffleboardLayout climberLayout = Dashboard
          .getDashboardTab(frc.robot.Constants.Dashboard.Tabs.DEBUG.tabName)
          .getLayout("Climber", BuiltInLayouts.kList).withSize(2, 4).withPosition(8, 0)
          .withProperties(Map.of("Label position", "TOP"));


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
    NamedCommands.registerCommand("Shoot First Note", new AutoOneNoteMiddle());
    NamedCommands.registerCommand("Pivot to Ground", new ProfiledPivotArmCmd(Constants.Arm.GROUND));
    NamedCommands.registerCommand("Intake", new IntakeCmdSeq(Constants.Intake.INTAKE_SPEED));
    NamedCommands.registerCommand("Pivot to Speaker", new ProfiledPivotArmCmd(Constants.Arm.SPEAKER));
    NamedCommands.registerCommand("Warm Up Shooter", new BangBangShooterCmd(Constants.Shooter.WARMUP_SPEAKER_RPM));
    NamedCommands.registerCommand("Bang Bang Shooter", new BangBangShooterCmd(Constants.Shooter.SPEAKER_RPM));
    NamedCommands.registerCommand("Run Intake Until RPM", new RunIntakeWhenShooterAtRPMCmd(Constants.Shooter.SPEAKER_RPM));
    NamedCommands.registerCommand("Hold Arm Speaker", new HoldArmInPositionCmd(Constants.Arm.SPEAKER));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("One Note Middle", new AutoOneNoteMiddle());

    /*
    autoChooser.addOption("Move Forward", new AutoMoveForward());
    autoChooser.addOption("Move Turn", new AutoTurn());

    
    autoChooser.addOption("Two Note Middle", new AutoTwoNoteMiddle());

    autoChooser.addOption("Three Note Middle", new StartAutoThreeNoteMiddle(ampOrEmpty, speakerOrAmp));
    autoChooser.addOption("Four Note Middle", new StartAutoFourNoteMiddle());

    autoChooser.addOption("One Note Amp Side ", new StartAutoOneNoteAmpSide());
    autoChooser.addOption("Two Note Amp Side", new StartAutoSpeakTwoNoteAmpSide());
    autoChooser.addOption("Two Note Speak Amp Side", new StartAutoSpeakTwoNoteAmpSide());

    autoChooser.addOption("One Note Empty Side", new StartAutoOneNoteEmptySide());
    autoChooser.addOption("Two Note Empty Side", new StartAutoTwoNoteEmptySide());
    */
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  return autoChooser.getSelected();
   // return new PathPlannerAuto("New Auto");
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