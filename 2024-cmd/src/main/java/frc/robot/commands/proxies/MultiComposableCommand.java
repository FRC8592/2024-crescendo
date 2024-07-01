package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MultiComposableCommand extends FunctionalCommand{
    /**
     * Wrapper command that does not register its command as a
     * composed command.
     *
     * @param command the command to wrap
     */
    public MultiComposableCommand(Command command){
        super(
            () -> command.initialize(),
            () -> command.execute(),
            (interrupted) -> command.end(interrupted),
            () -> command.isFinished(),
            command.getRequirements().toArray(new Subsystem[0])
        );
    }
}
