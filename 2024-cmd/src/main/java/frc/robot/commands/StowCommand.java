package frc.robot.commands;

import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.subsystems.elevator.Elevator;

public class StowCommand extends NewtonCommand {
    /**
     * Command to stow the robot. Does not automatically override other commands; use
     * {@link OverrideEverythingCommand} for that.
     *
     * @apiNote This command ends when {@link Elevator#isAtTargetPosition()} returns
     * {@code true}.
     */
    public StowCommand(){
        super(
            stopSubsystems(shooter.commands, intake.commands)
            .alongWith(elevator.commands.setStaticPositionCommand(0, 0))
        );
    }
}
