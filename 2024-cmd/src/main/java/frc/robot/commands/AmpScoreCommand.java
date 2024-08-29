package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.commands.proxies.NewtonCommand;

public class AmpScoreCommand extends NewtonCommand {
    /**
     * Command to score in the amp. Assumes the elevator is already in the amp
     * position (must be checked elsewhere).
     *
     * @apiNote This command ends after {@link SHOOTER#AMP_SCORE_TIME} seconds
     */
    public AmpScoreCommand(){
        super(
            shooter.commands.ampScoreCommand()
            .alongWith(leds.commands.singleColorCommand(LEDS.OFF))
        );

        // Keep the elevator and intake from doing anything funky while we score.
        addRequirements(elevator, intake);
    }
}
