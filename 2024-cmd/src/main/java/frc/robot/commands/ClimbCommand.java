package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.elevator.ElevatorCommands;

public class ClimbCommand extends NewtonCommand {
    /**
     * Command to go to the climb position and hold. Runs a
     * {@link ElevatorCommands#setMalleablePositionCommand(double, double)}, so
     * {@link ElevatorCommands#incrementElevatorPositionCommand(double, double)}
     * will move the elevator.
     *
     * @apiNote This command never ends on its own; it must be interrupted
     * to end.
     */
    public ClimbCommand(){
        super(
            stopSubsystems(shooter.commands).andThen(
                elevator.commands.setMalleablePositionCommand(
                    ELEVATOR.PIVOT_ANGLE_MAX,
                    ELEVATOR.EXTENSION_METERS_MAX
                )
            )
        );
    }
}
