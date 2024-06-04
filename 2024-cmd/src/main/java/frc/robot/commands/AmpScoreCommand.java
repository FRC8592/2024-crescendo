package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class AmpScoreCommand extends WrapperCommand {
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
            shooter.commands.ampScoreCommand()
            .alongWith(leds.commands.singleColorCommand(LEDS.OFF))
        );
    }
}
