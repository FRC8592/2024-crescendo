package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.proxies.*;

public class ShootCommand extends NewtonCommand {
    /**
     * Command that, assuming the robot is primed (this is NOT checked), shoots
     * the currently loaded note.
     *
     * @apiNote This command runs for {@link SHOOTER#SHOOT_SCORE_TIME} seconds.
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     */
    public ShootCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            shooter.fireCommand()
            .alongWith(leds.singleColorCommand(LEDS.OFF))
        );
    }
}
