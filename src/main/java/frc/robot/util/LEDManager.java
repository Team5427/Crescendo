package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDManager {

    public static enum LED_State {
        kPickedUp,
        kHasGamePiece,
        kWantsGamePiece,
        kIdle
    }

    private static final Color LIMELIGHT_GREEN = new Color(0, 155, 0);

    private static final int ledPort = 0;

    private static AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;
    private static int ledCount;

    private static double updateDelay;

    private static LED_State ledState;

    private static int tick;

    public LEDManager() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);

        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start();

        updateDelay = 0.02;
        tick = 0;

        ledState = LED_State.kIdle;
    }

    public static void setState(LED_State state) {
        ledState = state;
    }

    public static LED_State getState() {
        return ledState;
    }

    private static void fillLED() {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer.setLED(i, LIMELIGHT_GREEN);
        }
    }

    public static void updateManager() {
        
    }
    
}
