package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.*;

public class ClimbCommand extends ProxyCommand {
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
