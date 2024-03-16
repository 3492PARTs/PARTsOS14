package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.util.Color;

public class LEDState extends SubsystemBase {
    public static LEDState ledState;

    AddressableLED armLEDs = new AddressableLED(Constants.LED.LED_PORT);
    AddressableLEDBuffer armBuffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);

    enum ColorName {
        RED,
        YELLOW,
        GREEN,
        DEFAULT // Orange and Blue
    }

    public LEDState() {
        armLEDs.setLength(armBuffer.getLength());
        armLEDs.setData(armBuffer);
        switchColors(ColorName.DEFAULT);
        armLEDs.start();
    }

    public static LEDState getInstance() {
        if (ledState == null) {
            ledState = new LEDState();
        }
        return ledState;
    }

    public void switchColors(ColorName colorName) {
        switch(colorName) {
            case RED:
                // Arm on ground, solid red.
                for (var i = 0; i < armBuffer.getLength(); i++) {
                    armBuffer.setRGB(i, 255, 0, 0);
                }
                break;
            case GREEN:
                // Note sensed.
                for (var i = 0; i < armBuffer.getLength(); i++) {
                    armBuffer.setRGB(i, 0, 255, 0);
                }
                break;
            case YELLOW:
                // An ERROW has occurred. Blink.
                errorColor();
                break;
            case DEFAULT:
                // Orange and Blue thing.
                defaultColor();
                break;
            default:
                // Orange and Blue thing.
                defaultColor();
                break;
        }
    }

    /** Error Color
     * Sets LEDs to yellow, then to black.<p>
     * Call this every periodic.<p>
     * Make sure to limit it though.
     */
    private void errorColor() {
        // Get LED, if it's yellow then switch to black for blinking.
        if (armBuffer.getLED(0).equals(Color.kYellow)) {
            for (var i = 0; i < armBuffer.getLength(); i++) {
                armBuffer.setRGB(i, 0, 0, 0);
            }
        // Vice Versa...
        } else {
            for (var i = 0; i < armBuffer.getLength(); i++) {
                armBuffer.setRGB(i, 255, 255, 0);
            }
        }
    }

    /** Default Color
     * Sets LEDs to Orange, then to blue.<p>
     * Call this every periodic.<p>
     * Make sure to limit it though.
     */
    private void defaultColor() {
        // Get LED, if it's orange then switch to blue back and forth.
        if (armBuffer.getLED(0).equals(Color.kOrange)) {    
            for (var i = 0; i < armBuffer.getLength(); i++) {
                armBuffer.setRGB(i, 0, 0, 255);
            }
        // Vice Versa...
        } else {
            for (var i = 0; i < armBuffer.getLength(); i++) {
                armBuffer.setRGB(i, 255, 140, 0);
            }
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        if (System.currentTimeMillis()%20 == 0) {
            switchColors(ColorName.DEFAULT);
        }
    }
}
