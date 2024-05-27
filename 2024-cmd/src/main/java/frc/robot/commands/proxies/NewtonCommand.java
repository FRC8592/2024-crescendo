package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NewtonCommand extends ProxyCommand {
    /**
     * Proxy command that requires the same subsystems as the passed-in command. Used
     * for creating a class out of an inline command build.
     *
     * @param command the command to run
     */
    public NewtonCommand(Command command){
        super(command);
        for(Subsystem s:command.getRequirements()){
            this.addRequirements(s);
        }
    }
}
