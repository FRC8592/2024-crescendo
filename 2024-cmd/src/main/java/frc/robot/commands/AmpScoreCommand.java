package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.elevator.Elevator.Positions;
import frc.robot.commands.proxies.NewtonCommand;

public class AmpScoreCommand extends NewtonCommand {
    /**
     * Command to prime for and score in the amp
     *
     * @param readyToScore a lambda that returns whether it's time to score.
     * Note that this command never scores before the elevator is in position;
     * this argument is used to add an extra wait condition (usually a button)
     * before scoring.
     *
     * @apiNote This command doesn't have a simple end condition; see the code
     * for details.
     */
    public AmpScoreCommand(BooleanSupplier readyToScore){
        super(
            stopSubsystems(shooter.commands).andThen(
                elevator.commands.setStaticPositionCommand(Positions.AMP)
                .andThen(
                    new WaitUntilCommand(readyToScore)
                )
                .andThen(
                    shooter.commands.ampScoreCommand()
                    .alongWith(leds.commands.singleColorCommand(LEDS.OFF))
                )
            )
        );
    }
}
