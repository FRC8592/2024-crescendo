package frc.robot.commands;

import frc.robot.Constants.*;

public class ShootCommand extends NewtonCommand {
    /**
     * Command that shoots the currently loaded note, assuming the robot is both
     * loaded and primed (neither of these are checked).
     *
     * @apiNote This command runs for {@link SHOOTER#SHOOT_SCORE_TIME} seconds.
     */
    public ShootCommand(){
        super(
            // Runs the feeder motors
            shooter.commands.fireCommand()

            // singleColorCommand is instant for now, but the deadline ensures
            // that nothing will break if that changes.
            .deadlineWith(leds.commands.singleColorCommand(LEDS.OFF))
        );

        // Don't allow the elevator and intake to be commanded to do anything funny
        // while shooting
        addRequirements(elevator, intake);
    }
}
