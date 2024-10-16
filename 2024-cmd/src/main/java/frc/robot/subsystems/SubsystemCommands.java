package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Class for methods that all subsystem command classes should
 * inherit or override
 */
public abstract class SubsystemCommands{
    /**
     * Default autonomous init command ({@link Commands#none()})
     */
    public Command autonomousInitCommand(){
        return Commands.none();
    }

    public abstract Command stopCommand();
}
