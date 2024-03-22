package frc.robot.subsystems.ErrorHandler;

import javax.management.RuntimeErrorException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ErrorManager extends SubsystemBase {
    public static ErrorManager errorManager;

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
     * Stops the robot dead in its tracks.<p>
     * Don't use this unless it's absolutely needed!
     */
    public void beanieBlock() {
        
    }
}
