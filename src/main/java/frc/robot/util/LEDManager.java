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

    private Color currentColor;
    private double colorFreq = Double.NaN;
    private Color setColor;
    private Timer loopTimer;

    private static final int ledPort = 0;
    private static final double updateDelay = 0.02;
    private static final double pickUpThreshold = 1.0;

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private static final int ledCount = 95;

    private LEDState ledState;

    private int tick;

    public LEDManager() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);

        led.setLength(ledCount);
        led.setData(ledBuffer);
        led.start();
        tick = 0;

        loopTimer = new Timer();

        ledState = LEDState.kDisabled;
        this.currentColor = Color.kDarkRed;
        setColor = Color.kDarkRed;


        loopTimer.start();
        instance = this;
    }

    public static LEDManager getInstance() {
        return instance;
    }

    public void setState(LEDState state) {
        ledState = state;
        // loopTimer.reset();
        // loopTimer.start();
    }

    public LEDState getState() {
        return ledState;
    }

    private void fillLED() {



        if (!Double.isNaN(colorFreq)) {
            if (loopTimer.get() >= (1.0 / colorFreq)) {
                if (setColor == Color.kBlack) {
                    this.setColor = currentColor;
                } else if (setColor == currentColor) {
                    this.setColor = Color.kBlack;
                } else {
                    this.setColor = currentColor;
                }
                loopTimer.reset();
                loopTimer.start();
            }
            // System.err.println(currentColor);
            // System.err.println(loopTimer.get());
        } else  {
            setColor = currentColor;
            loopTimer.reset();
        }

        for (int i = 0; i < ledCount; i++) {
            ledBuffer.setLED(i, setColor);
        }
        led.setData(ledBuffer);
    }

    public void updateManager() {
        // tick = ledState != LEDState.kPickedUp ? 0: tick;
        switch (ledState) {
            case kDisabled:
                this.currentColor = Color.kDarkRed;
                colorFreq = Double.NaN;
                break;

            case kEmpty:
                this.currentColor = Color.kGray;
                colorFreq = Double.NaN;
                break;

            case kIntaking:
                this.currentColor = Color.kGray;
                colorFreq = 20.0;
                break;
            
            case kIntakeFull:
                this.currentColor = Color.kBlue;
                colorFreq = Double.NaN;
                break;

            case kHandingOff:
                this.currentColor = Color.kBlue;
                colorFreq = 20.0;
                break;

            case kShooterLoaded:
                this.currentColor = Color.kDarkGreen;
                colorFreq = Double.NaN;
                break;

            case kTargeting:
                this.currentColor = Color.kDarkGreen;
                colorFreq = 20.0;
                break;
            
            case kAmpSignal:
                this.currentColor = Color.kYellow;
                colorFreq = 40.0;
                break;

            case kCoopSignal:
                this.currentColor = Color.kPurple;
                colorFreq = 40.0;
                break;

            default:
                this.currentColor = Color.kRed;
                colorFreq = Double.NaN;
                System.err.println("stuck on default");


        }

        fillLED();
    }

    @Override
    public void periodic() {
        updateManager();

        SteelTalonsLogger.post("LED State", ledState.name());
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
