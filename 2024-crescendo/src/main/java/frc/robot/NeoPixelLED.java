package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class NeoPixelLED {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private int LED_LENGTH = 30;
    public Timer flashTimer;
    private double counter = 0;

    public NeoPixelLED() {
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        ledStrip.start();
        flashTimer = new Timer();
        flashTimer.start();
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
    public void PARTY(){
        counter+=10;
        for(int i = 0; i < LED_LENGTH; i++) {
            int[] RGB = HSVtoRGB((counter+(i*5))%360);
            ledBuffer.setRGB(i, RGB[0], RGB[1], RGB[2]);
        }
        ledStrip.setData(ledBuffer);
    }
    // public void hone(){
        
    // }
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
