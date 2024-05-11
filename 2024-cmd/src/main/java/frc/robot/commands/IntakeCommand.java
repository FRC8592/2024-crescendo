package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.*;

public class IntakeCommand extends ProxyCommand {
    public IntakeCommand(Shooter shooter, Elevator elevator, Intake intake){
        super(
            new StowCommand(shooter, elevator, intake)
            .andThen(
                shooter.intakeNoContactCommand()
                .deadlineWith(intake.intakeCommand())
            )
            .andThen(
                shooter.intakeWithContactCommand()
                .deadlineWith(intake.intakeCommand())
            ).andThen(
                intake.stopCommand()
                .alongWith(shooter.intakeAdjustNoteCommand())
            )
        );
    }
}
