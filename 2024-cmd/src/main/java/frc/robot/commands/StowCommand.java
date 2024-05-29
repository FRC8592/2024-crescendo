package frc.robot.commands;

import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.proxies.*;

public class StowCommand extends NewtonCommand {
    /**
     * Command to stow the robot. Does not automatically override other commands; use
     * {@link OverrideEverythingCommand} for that
     *
     * @param shooter
     * @param elevator
     * @param intake
     */
    public StowCommand(Shooter shooter, Elevator elevator, Intake intake){
        super(
            shooter.stopCommand()
            .alongWith(intake.stopCommand())
            .alongWith(elevator.setStaticPositionCommand(0, 0))
        );
    }
}
