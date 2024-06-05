package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeCommand extends WrapperCommand {
    /**
     * Command to stow, then run the full intake routine
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     */
    public IntakeCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            // Make sure we're stowed
            new StowCommand(shooter, elevator, intake)

            // Run the intake and feeders as if there is no contact with
            // a note. This stopw when the intakeNoContactCommand ends,
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
