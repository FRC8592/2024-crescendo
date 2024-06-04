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
            new StowCommand(shooter, elevator, intake)
            .andThen(
                shooter.commands.intakeNoContactCommand()
                .deadlineWith(
                    intake.commands.intakeCommand()
                    .alongWith(leds.commands.singleColorCommand(LEDS.ORANGE))
                )
            )
            .andThen(
                shooter.commands.intakeWithContactCommand()
                .deadlineWith(
                    intake.commands.intakeCommand()
                    .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 2))
                )
            ).andThen(
                shooter.commands.intakeAdjustNoteCommand()
                .deadlineWith(
                    intake.commands.stopCommand()
                    .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 2))
                )
            )
        );
    }
}
