package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

public class AmpScoreCommand extends ProxyCommand {
    /**
     * Command to score in the amp. Assumes the elevator is already in the amp
     * position (must be checked elsewhere)
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     *
     * @apiNote This command ends after {@link SHOOTER#AMP_SCORE_TIME} seconds
     */
    public AmpScoreCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            shooter.ampScoreCommand()
            .alongWith(leds.singleColorCommand(LEDS.OFF))
        );
        this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
