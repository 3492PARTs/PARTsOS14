package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    Translation2d m_frontLeftLocation = new Translation2d(0.381, .381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, .381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -.381);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        init();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        init();
    }

    private void init() {
            Shuffleboard.getTab("Swerve Testing")
        .addNumber("Gyro", () -> { return getPigeon2().getAngle(); }).withWidget(BuiltInWidgets.kGyro).withPosition(0, 0);
        Shuffleboard.getTab("Swerve Testing")
        .addNumber("PoseX", () -> { return getPose().getX(); }).withWidget(BuiltInWidgets.kTextView).withPosition(0, 1);
        Shuffleboard.getTab("Swerve Testing")
        .addNumber("PoseY", () -> { return getPose().getY(); }).withWidget(BuiltInWidgets.kTextView).withPosition(0, 2);
        configurePP();
    }

    private void configurePP() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(.2, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(3, 0.0, 0.2), // Rotation PID constants
                    3.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command SnapToAngle(double newAngle){

        TrapezoidProfile turningProf = new TrapezoidProfile(new Constraints(1, 1));
        // Not the best way to do it, tradeoff for ease of readability.
        var currentState = new TrapezoidProfile.State(Math.toRadians(newAngle),0.0);
        var goalState = new TrapezoidProfile.State(0,0);
        
        Timer trapezoidTimer = new Timer();


        return this.runEnd(() -> {
            SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
            trapezoidTimer.start();
             this.setControl( drive.withRotationalRate(turningProf.calculate(trapezoidTimer.get(), currentState, goalState).velocity));
            System.out.println(turningProf.calculate(trapezoidTimer.get(), currentState, goalState).velocity);
        
        }, () -> {trapezoidTimer.reset();}

          // Drive counterclockwise with negative X (left)
        
        
       ).until( () -> turningProf.totalTime() <= trapezoidTimer.get() );

        }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        // m_pigeon2.setYaw(pose.getRotation().getDegrees());
        super.m_odometry.resetPosition(pose.getRotation(), m_modulePositions, pose);
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
}
