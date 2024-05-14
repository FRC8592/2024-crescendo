package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class PassThroughCommand extends ProxyCommand {
    /**
     * Command to stow, then run pass-through indefinitely
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public PassThroughCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            new StowCommand(shooter, elevator, intake)
            .andThen(
                intake.intakeCommand()
                .alongWith(shooter.passThroughCommand())
                .alongWith(leds.blinkCommand(LEDS.ORANGE, 4))
            )
        );
    }
}
