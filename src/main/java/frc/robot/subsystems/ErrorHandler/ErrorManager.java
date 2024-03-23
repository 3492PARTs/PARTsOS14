package frc.robot.subsystems.ErrorHandler;

import java.util.List;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ErrorManager extends SubsystemBase {
    public static ErrorManager errorManager;

    public List<Integer> motorErrors;

    public boolean AUTOS_DISABLED = false;
    public boolean motorHasError = false;

    // Local pointers to modules.
    private Arm arm;
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    

    public enum ErrorType {
        MOTOR_ERROR(0), // Error for motors.
        LED_ERROR(1), // Issue with LEDs
        INIT_ERROR(2), // Error with Robot or Robot Container.
        SYS_ERROR(3), // From RoboRIO issues to WPILib issues.
        UNHANDLED_ERROR(4); // Catch all for other errors.

        public int code;

        ErrorType(int code) {
            this.code = code;
        }
    }

    public enum ArmError {
        DID_NOT_REACH_SETPOINT(0),
        CAN_NOT_HOLD(1),
        LOST_CONTROL(2);

        public int code;

        ArmError(int code) {
            this.code = code;
        }
    }

    public enum MiscError {
        LIMIT_SWITCH(0),
        LED_ERROR(1);

        public int code;

        MiscError(int code) {
            this.code = code;
        }
    }

    public ErrorManager() {

    }

    /**
     * The method that starts telemetry and error reporting.<p>
     * Use this in Robot.java first, before anything else.
     */
    public void start() {
        arm = Arm.getInstance();
        driveTrain = DriveTrain.getInstance();
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();

    }

    /**
     * Gets and returns the instance of ErrorManager.<p>
     * If instance does not exist, it makes one.
     */
    public static ErrorManager getInstance() {
        if (errorManager == null) {
            errorManager = new ErrorManager();
        }
        return errorManager;
    }

    /**
     * The error handler method for catch all and misc. terms.<p>
     * 
     * @param msg - The error message to print to the console / the RioLog.
     * @param errorType - The error type to handle.
     */
    public void handle(String msg, ErrorType errorType) {
        switch (errorType) {
            case INIT_ERROR:
                break;
            
            case SYS_ERROR:
                break;

            case MOTOR_ERROR:
                break;
            
            case LED_ERROR:
                break;
        
            default:
                // Unhandled Error
                // Stack trace moment.
                throw new RuntimeException(msg);
        }
    }

    /**
     * The error handler method for all motor errors.<p>
     * 
     * @param msg - The error message to print to the console / the RioLog.
     * @param errorType - The error type to handle. (Motor)
     * @param motorError - The error from the motor.
     */  
    public void handle(String msg, ErrorType errorType, REVLibError motorError) {
        // Motor error handler.
        if (errorType == ErrorType.MOTOR_ERROR) {
            System.out.println(msg + "\nThe error code from motor:\n" + motorError.toString());

        }
    }

    public void handleArm(ArmError error) {

    }

    /**
     * Stops the robot dead in its tracks.<p>
     * Don't use this unless it's absolutely needed!
     */
    public void beanieBlock() {
        
    }
}