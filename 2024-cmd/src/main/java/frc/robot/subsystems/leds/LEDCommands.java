package frc.robot.subsystems.leds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.subsystems.SubsystemCommands;

public class LEDCommands extends SubsystemCommands{
    private LEDs leds;
    public LEDCommands(LEDs leds){
        this.leds = leds;
    }

    /**
     * Command that turns the light strip cyan or off depending on whether there is a
     * note loaded as determined by the passed-in lambda.
     *
     * @param isLoaded lambda that returns whether there is a note in the robot.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command indicateLoadedCommand(BooleanSupplier isLoaded){
        return leds.run(() -> {
            // If loaded, show cyan lights; otherwise, do nothing
            leds.setSolidColor(isLoaded.getAsBoolean() ? LEDS.CYAN : LEDS.OFF);
            leds.write();
        });
    }

    /**
     * Command to turn the whole light-strip a certain color
     *
     * @param color the color the strip should become
     *
     * @return the command
     *
     * @apiNote This command runs instantly and ends on the same frame
     */
    public Command singleColorCommand(Color color){
        return leds.runOnce(() -> {
            leds.setSolidColor(color);
            leds.write();
        });
    }

    /** @return {@link Commands#none()}; stopCommand() is not applicable to LEDs */
    public Command stopCommand(){return Commands.none();}

    /**
     * Command to blink the lights at a certain color with the specified speed
     *
     * @param color the color the lights should be
     * @param hertz the frequency in hertz that the lights should blink
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command blinkCommand(Color color, int hertz){
        Timer flashTimer = new Timer();
        flashTimer.start();

        return leds.run(() -> {
            if (((int) (flashTimer.get() * hertz * 2)) % 2 == 0) {
                leds.setSolidColor(color);
            }
            else {
                leds.setSolidColor(LEDS.OFF);
            }
            leds.write();
        });
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
        return leds.run(() -> {
            counter[0]+=10;
            for(int i = 0; i < LEDS.LED_LENGTH; i++) {
                int[] RGB = leds.HSVtoRGB((counter[0]+(i*5))%360);
                leds.setLED(i, new Color(RGB[0], RGB[1], RGB[2]));
            }
            leds.write();
        });
    }

    /**
     * Command to run the honing lights (show drivers how far off they are)
     *
     * @param offset lambda that returns how far off-center the target is.
     * Usually a call to PoseVision.offsetFromAprilTag()
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted to end
     */
    public Command honeCommand(DoubleSupplier offset){
        return leds.run(() -> {
            // If we're completely off-aim, just show red lights
            if (offset.getAsDouble() >= LEDS.NOT_AIMED_OFFSET){
                leds.setSolidColor(LEDS.RED);
            }

            // Otherwise (if we're not off-aim), if we're too far off to be fully aimed, show the honing lights
            else if (offset.getAsDouble() > LEDS.FULLY_AIMED_OFFSET){
                leds.setSolidColor(LEDS.RED);
                double ledOffsetScale = 1 - ((offset.getAsDouble() - LEDS.FULLY_AIMED_OFFSET)
                        /(LEDS.NOT_AIMED_OFFSET - LEDS.FULLY_AIMED_OFFSET)); //a value of 0-1 where 0 means equal to NOT_AIMED_OFFSET and
                                                                            //1 means FULLY_AIMED_OFFSET
                double ledsToLight = ledOffsetScale * (LEDS.LED_LENGTH/2);
                for (int i = 0; i < ledsToLight; i ++){
                    leds.setLED(i, LEDS.GREEN);
                    leds.setLED(LEDS.LED_LENGTH - i - 1, LEDS.GREEN);
                }
            }

            // If we're fully aimed, simply show green
            else{
                leds.setSolidColor(LEDS.GREEN);
            }
            leds.write();
        });
    }
}
