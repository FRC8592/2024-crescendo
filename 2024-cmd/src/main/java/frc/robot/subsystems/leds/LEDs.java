package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.NewtonSubsystem;

import frc.robot.Constants.*;

public class LEDs extends NewtonSubsystem {
    private static LEDs instance = null;
    public static LEDs getInstance(){
        if(instance == null){
            throw new IllegalStateException("The LEDs subsystem must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static LEDs instantiate(){
        if(instance != null){
            throw new IllegalStateException("The LEDs subsystem can't be instantiated twice");
        }
        instance = new LEDs();
        return instance;
    }

    public LEDCommands commands = new LEDCommands(this);

    // Represents the real strip. Should only be written to once per frame.
    private AddressableLED ledStrip;

    // An array of color data that's separate from the strip. This can be
    // edited as much as you want, and must be manually written to the strip
    // for the LEDs to display anything.
    private AddressableLEDBuffer ledBuffer;

    private LEDs() {
        ledStrip = new AddressableLED(LEDS.LED_STRIP_PWM_PORT);
        ledStrip.setLength(LEDS.LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(LEDS.LED_LENGTH);
        ledStrip.start();
    }

    /**
     * Set the entire LED strip to a color (doesn't write to the physical
     * strip; see {@link LEDs#write()})
     *
     * @param color the color to set the strip to
     */
    protected void setSolidColor(Color color){
        for(int i = 0; i < LEDS.LED_LENGTH; i++){
            ledBuffer.setLED(i, color);
        }
    }

    public void periodic(){}
    public void simulationPeriodic(){}

    /**
     * Set an individual LED to a specific color (doesn't write to the
     * strip; see {@link LEDs#write()})
     *
     * @param index the index of the LED to set
     * @param color the color to set the LED to
     */
    protected void setLED(int index, Color color){
        ledBuffer.setLED(index, color);
    }

    /**
     * Write the buffer data to the LED strip
     */
    protected void write(){
        ledStrip.setData(ledBuffer);
    }

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

    // Not applicable to LEDs, but required by NewtonSubsystem
    protected void stop(){}
}