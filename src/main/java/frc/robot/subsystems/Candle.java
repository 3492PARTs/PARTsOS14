// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;

public class Candle extends SubsystemBase {
    private final CANdle candle;
    private final int LED_LENGTH = Constants.LED.LED_LENGTH;
    private Animation animation = null;

    private static Candle candleInstance;

    public enum Color {

        RED(254, 0, 0),
        ORANGE(254, 55, 0),
        YELLOW(254, 254, 0),
        GREEN(0, 254, 0),
        BLUE(0, 0, 254),
        PURPLE(118, 0, 254),
        WHITE(254, 254, 254),
        OFF(0, 0, 0);

        public int r;
        public int g;
        public int b;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    /** Creates a new light. */
    private Candle() {
        candle = new CANdle(Constants.LED.LED_PORT, "rio");
        candle.configBrightnessScalar(.5);
        candle.configLEDType(LEDStripType.RGB);
        setColor(Color.OFF);
    }

    public static Candle getInstance() {
        // If instance is null, then make a new instance.
        if (candleInstance == null) {
            candleInstance = new Candle();
        }
        return candleInstance;
    }

    public void setColor(Color color) {
        animation = null;
        candle.animate(animation);
        candle.setLEDs(color.r, color.g, color.b);
    }

    public void setNoColor() {
        setColor(Color.OFF);
    }

    public Command setColorGreenCommand() {
        return runOnce(() -> setColor(Color.GREEN));
    }

    public Command setNoColorCommand() {
        return runOnce(() -> setColor(Color.OFF));
    }

    public FireAnimation getBurnyBurnAnimation() {
        return new FireAnimation(1, .5, LED_LENGTH, .5, .5);
    }

    public RainbowAnimation getRainbowAnimation() {
        return new RainbowAnimation();
    }

    public StrobeAnimation getBlinkAnimation(Color color) {
        return new StrobeAnimation(color.r, color.g, color.b);
    }

    public SingleFadeAnimation getFadeAnimation(Color color) {
        return new SingleFadeAnimation(color.r, color.g, color.b);
    }

    public void runBurnyBurnAnimation() {
        animation = getBurnyBurnAnimation();
    }

    public void runRainbowAnimation() {
        animation = getRainbowAnimation();
    }

    public void runBlinkAnimation(Color color) {
        animation = getBlinkAnimation(color);
    }

    public void runFadeAnimation(Color color) {
        animation = getFadeAnimation(color);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (animation != null) {
            candle.animate(animation);
        }
    }
}
