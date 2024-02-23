package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class LEDManager {

    public static enum LED_State {
        kPickedUp,
        kHasGamePiece,
        kWantsGamePiece,
        kIdle
    }

    private static final Color LED_OFF = new Color(0, 0, 0);
    private static final Color LIMELIGHT_GREEN = new Color(0, 155, 0);
    private static final Color RED_ALLIANCE = new Color(255, 0, 0);
    private static final Color BLUE_ALLIANCE = new Color(0, 0, 255);

    private static Color currentColor;

    private static final int ledPort = 0;
    private static final double updateDelay = 0.02;
    private static final double pickUpThreshold = 1.0;

    private static AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;
    private static int ledCount;

    private static LED_State ledState;

    private static int tick;

    public LEDManager() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);

        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start();
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
            ledBuffer.setLED(i, currentColor);
        }
    }

    public static void updateManager() {
        Alliance alliance = DriverStation.getAlliance().get();
        switch (ledState) {
            case kPickedUp:
                if (tick % 0.2 == 0)
                    currentColor = currentColor.equals(LIMELIGHT_GREEN) ? LED_OFF: LIMELIGHT_GREEN;
                tick += updateDelay;
                if (tick > pickUpThreshold)
                    setState(LED_State.kHasGamePiece);
                break;
            case kHasGamePiece:
                currentColor = LIMELIGHT_GREEN;
            case kWantsGamePiece:
                currentColor = alliance == Alliance.Red ? RED_ALLIANCE: BLUE_ALLIANCE;
            default:
                currentColor = LED_OFF;
        }

        fillLED();
    }
    
}
