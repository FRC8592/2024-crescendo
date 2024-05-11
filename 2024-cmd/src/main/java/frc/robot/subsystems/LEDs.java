package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class LEDs extends SubsystemBase{
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    public Timer flashTimer;

    public LEDs() {
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(LEDS.LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LEDS.LED_LENGTH);
        ledStrip.start();
        flashTimer = new Timer();
        flashTimer.start();
    }

    public Command defaultCommand(){
        return run(() -> {
            setSolidColor(new Color(0,0,0));
        });
    }

    public Command singleColorCommand(Color color){
        return runOnce(() -> {
            setSolidColor(color);
        });
    }

    public Command blinkCommand(Color color, int hertz){
        return run(() -> {
            if (((int) (flashTimer.get() * hertz * 2)) % 2 == 0) {
                setSolidColor(color);
            }
            else {
                setSolidColor(new Color(0,0,0));
            }
        });
    }
    
    public Command partyCommand(){
        // The counter variable is required to be final for some reason, so
        // put the editable value in a final array

        final int[] counter = {0};
        return run(() -> {
            counter[0]+=10;
            for(int i = 0; i < LEDS.LED_LENGTH; i++) {
                int[] RGB = HSVtoRGB((counter[0]+(i*5))%360);
                ledBuffer.setRGB(i, RGB[0], RGB[1], RGB[2]);
                ledStrip.setData(ledBuffer);
            }
        });
    }

    private void setSolidColor(Color color){
        for(int i = 0; i < LEDS.LED_LENGTH; i++){
            ledBuffer.setLED(i, color);
            ledStrip.setData(ledBuffer);
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

    public void periodic(){}
    public void simulationPeriodic(){}
}