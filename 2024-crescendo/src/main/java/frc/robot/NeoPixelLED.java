package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;

public class NeoPixelLED {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    public Timer flashTimer;
    private double offset;
    private boolean cameraWorks;

    public static class NewtonColor{
        public final int red;
        public final int green;
        public final int blue;
        public NewtonColor(int r, int g, int b){
            this.red = r;
            this.green = g;
            this.blue = b;
        }
        public int[] getColors(){
            return new int[] {red, green, blue};
        }
    }

    public NeoPixelLED() {
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(LEDS.LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LEDS.LED_LENGTH);
        ledStrip.start();
        flashTimer = new Timer();
        flashTimer.start();
    }

    public void update(double offset, boolean cameraWorks) {
        ledStrip.setData(ledBuffer);
        this.offset = offset;
        this.cameraWorks = cameraWorks;
    }

    public void solidColor(NewtonColor color){
        for(int i = 0; i < LEDS.LED_LENGTH; i++) {
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        }
    }

    public void blinkColor(NewtonColor blink, double hertz){
        if (((int) (flashTimer.get() * hertz)) % 2 == 0) {
            for (int i = 0; i < LEDS.LED_LENGTH; i++) {
                ledBuffer.setRGB(i, blink.red, blink.green, blink.blue);
            }
        }
        else {
            for (int i = 0; i < LEDS.LED_LENGTH; i++) {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }

    public void hone(){
        if(cameraWorks){
            if (offset >= LEDS.NOT_AIMED_OFFSET){
                solidColor(LEDS.RED);
            }
            else if (offset <= LEDS.NOT_AIMED_OFFSET && offset > LEDS.FULLY_AIMED_OFFSET){
                solidColor(LEDS.RED);
                double ledOffsetScale = 1 - ((offset - LEDS.FULLY_AIMED_OFFSET)
                        /(LEDS.NOT_AIMED_OFFSET - LEDS.FULLY_AIMED_OFFSET)); //a value of 0-1 where 0 means equal to NOT_AIMED_OFFSET and 
                                                                            //1 means FULLY_AIMED_OFFSET
                double ledsToLight = ledOffsetScale * (LEDS.LED_LENGTH/2);
                for (int i = 0; i < ledsToLight; i ++){
                    ledBuffer.setRGB(i, LEDS.GREEN.red, LEDS.GREEN.green, LEDS.GREEN.blue);
                    ledBuffer.setRGB(LEDS.LED_LENGTH - i - 1, LEDS.GREEN.red, LEDS.GREEN.green, LEDS.GREEN.blue);
                }
            }
            else{
                solidColor(LEDS.GREEN);
            }
        }
        else{
            solidColor(LEDS.RED);
        }
    }
}
