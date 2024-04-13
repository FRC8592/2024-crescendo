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
    private boolean cameraWorking;
    private int counter = 0;

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

    public void update(double offset, boolean cameraWorking) {
        ledStrip.setData(ledBuffer);
        this.offset = Math.abs(offset);
        this.cameraWorking = cameraWorking;
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
        if(this.cameraWorking){
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
            solidColor(LEDS.CYAN);
        }
    }
    public void PARTY(){
        counter+=10;
        for(int i = 0; i < LEDS.LED_LENGTH; i++) {
            int[] RGB = HSVtoRGB((counter+(i*5))%360);
            ledBuffer.setRGB(i, RGB[0], RGB[1], RGB[2]);
        }
    }
    private int[] HSVtoRGB(double h){
        double R1 = 0;
        double G1 = 0;
        double B1 = 0;
        if(0 <= h && h < 60){
            R1 = 255;
            G1 = (h-0)*(255d/60d);
            B1 = 0;
        }
        else if(60 <= h && h < 120){
            R1 = (120-h)*(255d/60d);
            G1 = 255;
            B1 = 0;
        }
        else if(120 <= h && h < 180){
            R1 = 0;
            G1 = 255;
            B1 = (h-120)*(255d/60d);
        }
        else if(180 <= h && h < 240){
            R1 = 0;
            G1 = (240-h)*(255d/60d);
            B1 = 255;
        }
        else if(240 <= h && h < 300){
            R1 = (h-240)*(255d/60d);
            G1 = 0;
            B1 = 255;
        }
        else if(300 <= h && h < 360){
            R1 = 255;
            G1 = 0;
            B1 = (360-h)*(255d/60d);
        }
        R1=Math.max(Math.min(R1, 255), 0);
        G1=Math.max(Math.min(G1, 255), 0);
        B1=Math.max(Math.min(B1, 255), 0);
        return new int[]{(int)R1, (int)G1, (int)B1};
    }
}
