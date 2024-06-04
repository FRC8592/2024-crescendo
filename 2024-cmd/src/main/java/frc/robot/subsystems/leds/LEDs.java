package frc.robot.subsystems.leds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.SubsystemCommands;

public class LEDs extends SubsystemBase {
    public LEDCommands commands = new LEDCommands(this);
    // Represents the real strip. Should only be written to once per frame.
    protected AddressableLED ledStrip;

    // An array of color data that's separate from the strip. This can be
    // edited as much as you want, and must be manually written to the strip
    // for the LEDs to display anything.
    protected AddressableLEDBuffer ledBuffer;

    public LEDs() {
        ledStrip = new AddressableLED(LEDS.LED_STRIP_PWM_PORT);
        ledStrip.setLength(LEDS.LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LEDS.LED_LENGTH);
        ledStrip.start();
    }

    protected void setSolidColor(Color color){
        for(int i = 0; i < LEDS.LED_LENGTH; i++){
            ledBuffer.setLED(i, color);
        }
        ledStrip.setData(ledBuffer);
    }

    public void periodic(){}
    public void simulationPeriodic(){}

    protected int[] HSVtoRGB(double h){
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