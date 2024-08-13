package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.elevator.Elevator;

public class ClimbCommand extends NewtonCommand {
    /**
     * Command to go to the climb position and hold. Runs a
     * {@link Elevator#setMalleablePositionCommand(double, double)}, so
     * {@link Elevator#incrementElevatorPositionCommand(double, double)}
     * will move the elevator.
     *
     * @apiNote This command never ends on its own; it must be interrupted
     * to end.
     */
    public ClimbCommand(){
        super(
            shooter.commands.stopCommand()
            .alongWith(intake.commands.stopCommand())
            .alongWith(
                elevator.commands.setMalleablePositionCommand(
                    ELEVATOR.PIVOT_ANGLE_MAX,
                    ELEVATOR.EXTENSION_METERS_MAX
                )
            )
        );
    }
}
