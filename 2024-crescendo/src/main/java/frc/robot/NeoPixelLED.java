package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class NeoPixelLED {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private int LED_LENGTH = 30;
    public Timer flashTimer;

    public NeoPixelLED() {
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        ledStrip.start();
        flashTimer = new Timer();
        flashTimer.start();
    }

    public void update() {
        ledStrip.setData(ledBuffer);
    }

    public void blinkYellow() {
        if (((int) (flashTimer.get() * 4)) % 2 == 0) {
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, 255, 255, 0);
            }
        }
        else {
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    public void blinkRed() {
        if (((int) (flashTimer.get() * 4)) % 2 == 0) {
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
        }
        else {
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    public void solidCyan() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 255,255);
        }
    }

    public void solidPink() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 192, 0, 255);
        }
    }

    public void solidOff() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void solidRed() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 255,0,0);
        }
    }
    public void solidGreen() {
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
    }
    public void solidOrange(){
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 255, 100, 0);
        }
    }
    public void solidBlue(){
        for(int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 0, 255);
        }
    }
}
