package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.elevator.Elevator.Positions;

public class PassThroughCommand extends NewtonCommand {
    /**
     * Command to run pass-through. Note that the robot will stow before starting
     * to pass-through.
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public PassThroughCommand(){
        super(
            elevator.commands.setStaticPositionCommand(Positions.AMP)
            .andThen(
                shooter.commands.passThroughCommand()
                .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 4))
            )
        );
    }
}
