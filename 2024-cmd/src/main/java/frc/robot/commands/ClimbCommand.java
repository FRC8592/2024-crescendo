package frc.robot.commands;

import frc.robot.Constants.ELEVATOR;
import frc.robot.commands.proxies.*;
import frc.robot.subsystems.*;

public class ClimbCommand extends NewtonCommand {
    /**
     * Command to go to the climb position and hold. Runs a
     * {@link Elevator#setMalleablePositionCommand(double, double)}, so
     * {@link Elevator#incrementElevatorPositionCommand(double, double)}
     * will make the elevator move.
     *
     * @param elevator
     * @param intake
     * @param shooter
     *
     * @apiNote This command never ends on its own; it must be interrupted
     * to end.
     */
    public ClimbCommand(Elevator elevator, Intake intake, Shooter shooter){
        super(
            shooter.stopCommand()
            .alongWith(intake.stopCommand())
            .alongWith(
                elevator.setMalleablePositionCommand(
                    ELEVATOR.PIVOT_ANGLE_MAX,
                    ELEVATOR.EXTENSION_METERS_MAX
                )
            )
        );
    }
}
