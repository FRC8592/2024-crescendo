package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class NeoPixelLED {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private int LED_LENGTH = 30;
    private int ledCounter = 0;
    private LEDMode mode;
    public Timer flashTimer;

    public NeoPixelLED() {
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        ledStrip.start();
        mode = LEDMode.OFF;
        flashTimer = new Timer();
        flashTimer.start();
    }

    public void update() {
        switch (mode) {
            case OFF:
                off();
                break;
            case AMP:
                amp();
                break;
            case NOTE_PICKUP:
                notePickup();
                break;
        }
    }

    public void amp() {
        if (((int) (flashTimer.get() * 4)) % 2 == 0) {
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, 255, 255, 0);
            }
            ledStrip.setData(ledBuffer);
        }
        else {
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
            ledStrip.setData(ledBuffer);
        }
    }

    public void notePickup() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 255,255);
        }
        ledStrip.setData(ledBuffer);
        
        ledCounter++;
        if (ledCounter > 200) {
            setMode(LEDMode.OFF);
        }
    }

    public void off() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        ledStrip.setData(ledBuffer);
    }

    public void red() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 255,0,0);
        }
        ledStrip.setData(ledBuffer);
    }
    
    public void disabled() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
        ledStrip.setData(ledBuffer);
    }

    static enum LEDMode {
        OFF,
        AMP,
        NOTE_PICKUP
    }

    public void setMode(LEDMode mode) {
        this.mode = mode;
        ledCounter = 0;
    }
}
