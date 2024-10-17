package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.elevator.Elevator.Positions;

public class IntakeCommand extends NewtonCommand {
    /**
     * Command to go to amp, then run the modified intake routine
     *
     * @apiNote This command doesn't take a predetermined
     * amount of time. See the source code for details
     */
    public IntakeCommand(){
        super(
            stopSubsystems(shooter.commands, intake.commands).andThen(
                elevator.commands.setStaticPositionCommand(Positions.AMP)
                .andThen(
                    shooter.commands.intakeNoteCommand()
                    .deadlineWith(
                        leds.commands.singleColorCommand(LEDS.ORANGE)
                    )
                ).andThen(
                    shooter.commands.intakeAdjustNoteCommand()
                    .deadlineWith(
                        intake.commands.stopCommand()
                        .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 2))
                    )
                )
            )
        );
    }
}
