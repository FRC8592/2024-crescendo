package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.NewtonCommand;

public class PassThroughCommand extends NewtonCommand {
    /**
     * Command to run pass-through. Note that the robot will stow before starting
     * to pass-through.
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public PassThroughCommand(){
        super(
            // Make sure we're stowed before we run anything
            new StowCommand()

            // Reusing the intake's intakeCommand may or may not be permanent
            .andThen(
                intake.commands.intakeCommand()
                .alongWith(shooter.commands.passThroughCommand())
                .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 4))
            )
        );
    }
}
