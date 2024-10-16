package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;

public class OutakeCommand extends NewtonCommand {
    /**
     * Command to outake. Note that this command is independent of what the
     * elevator might be doing.
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public OutakeCommand(){
        super(
            stopSubsystems(elevator.commands).alongWith(
                shooter.commands.outakeCommand()
                .alongWith(intake.commands.outakeCommand())
            )
        );
    }
}
