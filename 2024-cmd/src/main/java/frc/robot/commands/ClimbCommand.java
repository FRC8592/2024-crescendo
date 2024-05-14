package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.*;

public class ClimbCommand extends ProxyCommand {
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
        this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
