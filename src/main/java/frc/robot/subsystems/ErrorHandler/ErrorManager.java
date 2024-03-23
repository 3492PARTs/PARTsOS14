package frc.robot.subsystems.ErrorHandler;

import java.util.List;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ErrorManager extends SubsystemBase {
    public static ErrorManager errorManager;

    public List<Integer> motorErrors;

    public enum ErrorType {
        MOTOR_ERROR, // Error for motors.
        LED_ERROR, // Issue with LEDs
        INIT_ERROR, // Error with Robot or Robot Container.
        SYS_ERROR, // From RoboRIO issues to WPILib issues.
        UNHANDLED_ERROR // Catch all for other errors.
    }

    public ErrorManager() {

    }

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
            switch (motorError) {
                case kSetpointOutOfRange:
                    break;
            
                default:
                    break;
            }
            System.out.println(msg + "\nThe error code from motor:\n" + motorError.toString());
        }
    }

    /**
     * Stops the robot dead in its tracks.<p>
     * Don't use this unless it's absolutely needed!
     */
    public void beanieBlock() {
        
    }
}