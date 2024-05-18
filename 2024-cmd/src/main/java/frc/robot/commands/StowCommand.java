package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.subsystems.*;

public class StowCommand extends ProxyCommand {
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
            shooter.stopCommand()
            .alongWith(intake.stopCommand())
            .alongWith(elevator.setStaticPositionCommand(0, 0))
        );
        this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
