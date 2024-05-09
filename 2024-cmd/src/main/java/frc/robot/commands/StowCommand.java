package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.*;

public class StowCommand extends ProxyCommand {
    public StowCommand(Shooter shooter, Elevator elevator, Intake intake){
        super(shooter.stopCommand()
                .alongWith(intake.stopCommand())
                .alongWith(elevator.setElevatorPositionCommand(0, 0))
        );
    }
}
