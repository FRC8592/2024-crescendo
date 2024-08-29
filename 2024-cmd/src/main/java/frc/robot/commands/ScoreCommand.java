package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreCommand extends NewtonCommand{
    /**
     * Command to score in the speaker or the amp, depending on what the elevator is doing. Should have a
     * {@code whileTrue} trigger for the button; shoot and amp-score commands are scheduled instead of
     * wrapped, so they won't be cancelled if the driver lets go of the trigger during a score.
     *
     * @param forceShoot a lambda that returns whether to force-shoot (if shooting). Force-shoot uses an
     * {@link OverrideEverythingCommand}.
     *
     * @apiNote This command ends when the appropriate scoring action has been completed; see the
     * {@link ScoreCommand} class for more details.
     */
    public ScoreCommand(BooleanSupplier forceShoot){
        /*
         * We want this command to handle amp scoring and shooting. To do that, we use a cancel-on-false
         * trigger (manifested as a whileTrue in RobotContainer) and override it here where necessary.
         * This works by making sure the cancel-on-false is applied to a sequential WaitUntilCommand +
         * ScheduleCommand instead of the actual command that runs the robot.
         *
         * The resulting behavior: if the driver presses and releases the button before the robot is ready,
         * the WaitUntilCommand is cancelled and nothing happens. However, if the driver lets go of the button
         * while scoring, the scoring command will be allowed to finish (since it was scheduled independently).
         */
        super(
            new DeferredCommand(
                () -> { // <-- Notice that we're starting a lambda here
                    if(elevator.isTargeting(Elevator.Positions.AMP)){

                        // This is the first instance of the logic described above
                        return new WaitUntilCommand(
                            () -> elevator.isAtPosition(Elevator.Positions.AMP)
                        ).andThen(new ScheduleCommand(
                            new AmpScoreCommand()
                        ));
                    }
                    else{ // If the elevator isn't at (or going to) the amp position, run logic for shooting instead
                        if(forceShoot.getAsBoolean()){ // If we're force-shooting

                            // Ignore whatever else might have been happening and just shoot
                            return new OverrideEverythingCommand(
                                new ShootCommand()
                            );
                        }
                        else{
                            // This WaitUntilCommand has similar logic to the one above
                            return new WaitUntilCommand(
                                () -> shooter.readyToShoot() && elevator.isAtTargetPosition()
                            ).andThen(new ScheduleCommand(
                                new ShootCommand().andThen(new StowCommand())
                            ));
                        }
                    }
                },
                Set.of(shooter, elevator, intake, leds) // Requirements for this command
            )
        );
    }
}
