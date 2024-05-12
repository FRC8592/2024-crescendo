package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

public class AmpScoreCommand extends ProxyCommand {
    public AmpScoreCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            shooter.ampScoreCommand()
            .alongWith(leds.singleColorCommand(LEDS.OFF))
        );
        this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
