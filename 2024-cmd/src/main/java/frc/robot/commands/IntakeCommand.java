package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;

public class IntakeCommand extends WrapperCommand {
    /**
     * Command to run the full intake routine
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     */
    public IntakeCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            new StowCommand(shooter, elevator, intake)
            .andThen(
                shooter.intakeNoContactCommand()
                .deadlineWith(
                    intake.intakeCommand()
                    .alongWith(leds.singleColorCommand(LEDS.ORANGE))
                )
            )
            .andThen(
                shooter.intakeWithContactCommand()
                .deadlineWith(
                    intake.intakeCommand()
                    .alongWith(leds.blinkCommand(LEDS.ORANGE, 2))
                )
            ).andThen(
                shooter.intakeAdjustNoteCommand()
                .deadlineWith(
                    intake.stopCommand()
                    .alongWith(leds.blinkCommand(LEDS.ORANGE, 2))
                )
            )
        );
    }
}
