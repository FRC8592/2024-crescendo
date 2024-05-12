package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class ShootCommand extends ProxyCommand {
    public ShootCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            shooter.fireCommand()
            .alongWith(leds.singleColorCommand(LEDS.OFF))
        );
        this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
