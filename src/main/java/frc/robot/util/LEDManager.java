package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.managing.SubsystemManager;

public class LEDManager extends SubsystemBase {

    public static enum LED_State {
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

    private Color currentColor;

    private static final int ledPort = 0;
    private static final double updateDelay = 0.02;
    private static final double pickUpThreshold = 1.0;

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int ledCount;

    private LED_State ledState;

    private int tick;

    public LEDManager() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);

        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start();
        tick = 0;

        ledState = LED_State.kDisabled;

        instance = this;
    }

    public static LEDManager getInstance() {
        return instance;
    }

    public void setState(LED_State state) {
        ledState = state;
    }

    public LED_State getState() {
        return ledState;
    }

    private void fillLED() {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer.setLED(i, currentColor);
        }
    }

    public void updateManager() {
        // tick = ledState != LED_State.kPickedUp ? 0: tick;
        switch (ledState) {

        }

        fillLED();
    }

    @Override
    public void periodic() {
        //do all the update manager stuff here
    }
    
}
