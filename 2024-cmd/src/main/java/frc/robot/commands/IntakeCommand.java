package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.NewtonCommand;

public class IntakeCommand extends NewtonCommand {
    /**
     * Command to stow, then run the full intake routine
     *
     * @apiNote This command doesn't take a predetermined
     * amount of time. See the source code for details
     */
    public IntakeCommand(){
        super(
            // Make sure we're stowed
            new StowCommand()

            // Run the intake and feeders as if there is no contact with
            // a note. This stops when the intakeNoContactCommand ends,
            // which is when the bottom beam-break sees a note.
            .andThen(
                shooter.commands.intakeNoContactCommand()
                .deadlineWith(
                    intake.commands.intakeCommand()
                    .alongWith(leds.commands.singleColorCommand(LEDS.ORANGE))
                )
            )

            // Now that the robot knows there's a note, suck it up into the
            // shooter using motor control that assumes there is a note.
            .andThen(
                shooter.commands.intakeWithContactCommand()
                .deadlineWith(
                    intake.commands.intakeCommand()
                    .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 2))
                )
            )

            // Once the note is all the way at the top, adjust it to a good
            // position to shoot.
            .andThen(
                shooter.commands.intakeAdjustNoteCommand()
                .deadlineWith(
                    intake.commands.stopCommand()
                    .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 2))
                )
            )
        );
    }
}
