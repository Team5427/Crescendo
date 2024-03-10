package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class LEDManager extends SubsystemBase {


    public static enum LEDState {
        kDisabled, //static red
        kEmpty, //static white
        kIntaking, //flashing orange - 10 Hz
        kIntakeFull, //static blue
        kHandingOff, //flashing blue - 10 Hz
        kShooterLoaded, //static green
        kAmpSignal, //flashing yellow - 20 Hz
        kCoopSignal, //flashing purple - 20 Hz
        kTargeting //flashing green - 10 Hz
    }

    private static LEDManager instance;

    private static final int ledPort = 0;

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private static final int ledCount = 95;

    private LEDState ledState;

    private Timer tickTimer;
    private boolean statusBoolean;
    private boolean thresholdReached;
    private int tickCount;

    private static final Color LED_OFF = new Color(0, 0, 0);
    private static final Color LED_WHITE = new Color(255, 255, 255);
    private static final Color LED_RED = new Color(255, 0, 0);
    private static final Color LED_GREEN = new Color(0, 255, 0);
    private static final Color LED_BLUE = new Color(0, 0, 255);

    public LEDManager() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);

        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start();

        tickTimer = new Timer();
        statusBoolean = true;

        instance = this;
    }

    public static LEDManager getInstance() {
        return instance;
    }

    public void setState(LEDState state) {
        if (thresholdReached)
            ledState = state;
    }

    public LEDState getState() {
        return ledState;
    }

    @Override
    public void periodic() {
        switch (ledState) {
            case kDisabled:
                oscillate(LED_OFF, LED_RED, ledCount);
                break;
            case kIntakeFull:
                fillStrip(LED_BLUE);
                break;
            case kShooterLoaded:
                fillStrip(LED_GREEN);
                break;
            case kTargeting:
                blinkStrip(LED_GREEN, 5);
            default:
                oscillateStrip(LED_OFF, LED_WHITE);
                break;
        }
    }

    private void fillStrip(Color color) {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    private void oscillateStrip(Color firstColor, Color secondColor) {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer.setLED(i, oscillate(
                firstColor, secondColor, i)
            );
        }
    }

    private Color oscillate(Color firstColor, Color secondColor, int bufferIdx) {
        Color newColor;
        Color currentColor = ledBuffer.getLED(bufferIdx);
        boolean redGreater, blueGreater, greenGreater;
        redGreater = secondColor.red - firstColor.red > 0;
        blueGreater = secondColor.blue - firstColor.blue > 0;
        greenGreater = secondColor.green - firstColor.green > 0;
        if (statusBoolean) {
            newColor = new Color(
                currentColor.red + (currentColor.red % ((firstColor.red - secondColor.red) * 5)),
                currentColor.blue + (currentColor.blue % ((firstColor.blue - secondColor.blue) * 5)),
                currentColor.green + (currentColor.green % ((firstColor.green - secondColor.green) * 5))
            );

            boolean redComplete, blueComplete, greenComplete;
            redComplete = redGreater ? currentColor.red > secondColor.red: currentColor.red < secondColor.red;
            blueComplete = blueGreater ? currentColor.blue > secondColor.blue: currentColor.blue < secondColor.blue;
            greenComplete = greenGreater ? currentColor.green > secondColor.green: currentColor.green < secondColor.green;

            if (redComplete && blueComplete && greenComplete) {
                statusBoolean = false;
            }

        } else {
            newColor = new Color(
                currentColor.red - (currentColor.red % ((firstColor.red - secondColor.red) * 5)),
                currentColor.blue - (currentColor.blue % ((firstColor.blue - secondColor.blue) * 5)),
                currentColor.green - (currentColor.green % ((firstColor.green - secondColor.green) * 5))
            );

            boolean redComplete, blueComplete, greenComplete;
            redComplete = redGreater ? currentColor.red < secondColor.red: currentColor.red > secondColor.red;
            blueComplete = blueGreater ? currentColor.blue < secondColor.blue: currentColor.blue > secondColor.blue;
            greenComplete = greenGreater ? currentColor.green < secondColor.green: currentColor.green > secondColor.green;

            if (redComplete && blueComplete && greenComplete) {
                statusBoolean = true;
            }
        }

        return newColor;
    }

    private void blinkStrip(Color color, int thresholdCount) {
        if (tickCount < thresholdCount) {
            thresholdReached = false;
            if (tickTimer.get() > 0.025) {
                statusBoolean = !statusBoolean;
                thresholdCount += statusBoolean ? 1: 0;
            }
        } else {
            thresholdReached = true;
            thresholdCount = 0;
        }

        for (int i = 0; i < ledCount; i++) {
            if (statusBoolean) {
                ledBuffer.setLED(i, color);
            } else {
                ledBuffer.setLED(i, LED_OFF);
            }
        }
    }

    public void resetStates() {
        if (Shooter.getInstance().loaded()) {
            setState(LEDState.kShooterLoaded);
        } else if (Intake.getInstance().sensorCovered()) {
            setState(LEDState.kIntakeFull);
        } else {
            setState(LEDState.kEmpty);
        }
    }
    
}
