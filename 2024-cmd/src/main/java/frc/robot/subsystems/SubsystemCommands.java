package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemCommands extends SubsystemBase {
    /**
     * Command to run on autonomous init. Returns
     * {@link Commands#none()} if the subsystem doesn't
     * override it.
     */
    public Command autonomousInitCommand(){
        return Commands.none();
    }
}
