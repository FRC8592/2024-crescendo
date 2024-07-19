package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ClimbCommand extends WrapperCommand {
    /**
     * Command to go to the climb position and hold. Runs a
     * {@link Elevator#setMalleablePositionCommand(double, double)}, so
     * {@link Elevator#incrementElevatorPositionCommand(double, double)}
     * will move the elevator.
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
