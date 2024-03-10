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
    private int ledCount;

    private LEDState ledState;

    private Timer tickTimer;
    private boolean doIncrease;

    public LEDManager() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);

        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start();

        ledState = LEDState.kDisabled;

        tickTimer = new Timer();
        doIncrease = true;

        instance = this;
    }

    public static LEDManager getInstance() {
        return instance;
    }

    public void setState(LEDState state) {
        ledState = state;
    }

    public LEDState getState() {
        return ledState;
    }

    @Override
    public void periodic() {
        switch (ledState) {
            default:
                for (int i = 0; i < ledCount; i++) {
                    ledBuffer.setLED(i, oscillate(
                        new Color(0, 0, 0), new Color(255, 255, 255), i)
                    );
                }
                break;
        }
    }

    private Color oscillate(Color firstColor, Color secondColor, int bufferIdx) {
        Color newColor;
        Color currentColor = ledBuffer.getLED(bufferIdx);
        boolean redGreater, blueGreater, greenGreater;
        redGreater = secondColor.red - firstColor.red > 0;
        blueGreater = secondColor.blue - firstColor.blue > 0;
        greenGreater = secondColor.green - firstColor.green > 0;
        if (doIncrease) {
            newColor = new Color(
                currentColor.red + (currentColor.red % (firstColor.red - secondColor.red)),
                currentColor.blue + (currentColor.blue % (firstColor.blue - secondColor.blue)),
                currentColor.green + (currentColor.green % (firstColor.green - secondColor.green))
            );

            boolean redComplete, blueComplete, greenComplete;
            redComplete = redGreater ? currentColor.red > secondColor.red: currentColor.red < secondColor.red;
            blueComplete = blueGreater ? currentColor.blue > secondColor.blue: currentColor.blue < secondColor.blue;
            greenComplete = greenGreater ? currentColor.green > secondColor.green: currentColor.green < secondColor.green;

            if (redComplete && blueComplete && greenComplete) {
                doIncrease = false;
            }

        } else {
            newColor = new Color(
                currentColor.red - (currentColor.red % (firstColor.red - secondColor.red)),
                currentColor.blue - (currentColor.blue % (firstColor.blue - secondColor.blue)),
                currentColor.green - (currentColor.green % (firstColor.green - secondColor.green))
            );

            boolean redComplete, blueComplete, greenComplete;
            redComplete = redGreater ? currentColor.red < secondColor.red: currentColor.red > secondColor.red;
            blueComplete = blueGreater ? currentColor.blue < secondColor.blue: currentColor.blue > secondColor.blue;
            greenComplete = greenGreater ? currentColor.green < secondColor.green: currentColor.green > secondColor.green;

            if (redComplete && blueComplete && greenComplete) {
                doIncrease = true;
            }
        }

        return newColor;
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
