package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class NeoPixelLED {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private int LED_LENGTH = 30;
    private int ledCounter = 0;
    private LEDMode mode;

    public NeoPixelLED() {
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        ledStrip.start();
        mode = LEDMode.OFF;
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
        for (int i = 0; i < LED_LENGTH; i++) {
            if ((i + ledCounter / 5) % 8 < 4) {
                ledBuffer.setRGB(i, 255, 128, 0); // YELLOW
            } else {
                ledBuffer.setRGB(i, 0, 0, 0); // OFF
            }
        }
        ledCounter++;
        ledStrip.setData(ledBuffer);
        
        if (ledCounter > 200) {
            setMode(LEDMode.OFF);
        }
    }

    public void notePickup() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 80, 30);
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
