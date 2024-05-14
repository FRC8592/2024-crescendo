package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    /**
     * Command that turns the light strip cyan or off depending on whether there is a
     * note loaded as determined by the passed-in lambda.
     *
     * @param isLoaded {@code BooleanSupplier}: lambda that returns whether there
     * is a note in the robot.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command indicateLoadedCommand(BooleanSupplier isLoaded){
        return run(() -> {
            setSolidColor(isLoaded.getAsBoolean() ? LEDS.CYAN : LEDS.OFF);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    /**
     * Command to turn the whole light-strip a certain color
     *
     * @param color {@code Color}: the color the strip should become
     *
     * @return the command
     *
     * @apiNote This command runs instantly and ends on the same frame
     */
    public Command singleColorCommand(Color color){
        return runOnce(() -> {
            setSolidColor(color);
        });
    }

    /**
     * Command to blink the lights at a certain color with the specified speed
     *
     * @param color {@code Color}: the color the lights should be
     * @param hertz {@code int}: the frequency in hertz that the lights should blink
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command blinkCommand(Color color, int hertz){
        return run(() -> {
            if (((int) (flashTimer.get() * hertz * 2)) % 2 == 0) {
                setSolidColor(color);
            }
            else {
                setSolidColor(LEDS.OFF);
            }
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    /**
     * Command to run party mode (rainbow flashing)
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command partyCommand(){
        // The counter variable is required to be final for some reason, so
        // put the editable value in a final array

        final int[] counter = {0};
        return run(() -> {
            counter[0]+=10;
            for(int i = 0; i < LEDS.LED_LENGTH; i++) {
                int[] RGB = HSVtoRGB((counter[0]+(i*5))%360);
                ledBuffer.setRGB(i, RGB[0], RGB[1], RGB[2]);
            }
            ledStrip.setData(ledBuffer);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    /**
     * Command to run the honing lights (show drivers how far off they are)
     *
     * @param offset {@code DoubleSupplier}: lambda that returns how far
     * off-center the target is. Usually a call to PoseVision.offsetFromAprilTag()
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command honeCommand(DoubleSupplier offset){
        return run(() -> {
            if (offset.getAsDouble() >= LEDS.NOT_AIMED_OFFSET){
                setSolidColor(LEDS.RED);
            }
            else if (offset.getAsDouble() <= LEDS.NOT_AIMED_OFFSET && offset.getAsDouble() > LEDS.FULLY_AIMED_OFFSET){
                setSolidColor(LEDS.RED);
                double ledOffsetScale = 1 - ((offset.getAsDouble() - LEDS.FULLY_AIMED_OFFSET)
                        /(LEDS.NOT_AIMED_OFFSET - LEDS.FULLY_AIMED_OFFSET)); //a value of 0-1 where 0 means equal to NOT_AIMED_OFFSET and
                                                                            //1 means FULLY_AIMED_OFFSET
                double ledsToLight = ledOffsetScale * (LEDS.LED_LENGTH/2);
                for (int i = 0; i < ledsToLight; i ++){
                    ledBuffer.setLED(i, LEDS.GREEN);
                    ledBuffer.setLED(LEDS.LED_LENGTH - i - 1, LEDS.GREEN);
                }
            }
            else{
                setSolidColor(LEDS.GREEN);
            }
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private void setSolidColor(Color color){
        for(int i = 0; i < LEDS.LED_LENGTH; i++){
            ledBuffer.setLED(i, color);
        }
        ledStrip.setData(ledBuffer);
    }

    public void periodic(){}
    public void simulationPeriodic(){}

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