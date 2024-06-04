package frc.robot.commands;

import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class StowCommand extends WrapperCommand {
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
            shooter.commands.stopCommand()
            .alongWith(intake.commands.stopCommand())
            .alongWith(elevator.commands.setStaticPositionCommand(0, 0))
        );
    }
}
