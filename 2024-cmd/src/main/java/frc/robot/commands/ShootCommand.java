package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends WrapperCommand {
    /**
     * Command that, assuming the robot is primed (this is NOT checked), shoots
     * the currently loaded note (the command also doesn't check that there is
     * a note)
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     *
     * @apiNote This command runs for {@link SHOOTER#SHOOT_SCORE_TIME} seconds.
     */
    public ShootCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
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
