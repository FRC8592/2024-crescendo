package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.commands.proxies.WaitForConditionCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class ScoreCommand extends DeferredCommand{
    /**
     * Command to score in the speaker or the amp, depending on what the elevator is doing. Should have a
     * {@code whileTrue} trigger for the button; shoot and amp-score commands are scheduled instead of
     * wrapped, so they won't be cancelled if the driver lets go of the trigger during a score.
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     * @param forceShoot a lambda that returns whether to force-shoot (if shooting). Force-shoot uses an
     * {@link OverrideEverythingCommand}.
     *
     * @apiNote This command ends when the appropriate scoring action has been completed; see the
     * code for more details.
     */
    public ScoreCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds, BooleanSupplier forceShoot){
        /*
         * We want this command to handle amp scoring and shooting. To do that, we use a cancel-on-false
         * trigger (manifested as a whileTrue in RobotContainer) and override it here where necessary. The
         * way this works is by making sure the cancel-on-false is applied to a WaitForConditionCommand
         * instead of the actual command that runs the robot. Because the WaitForConditionCommand schedules
         * the passed-in command instead of wrapping it, the result is that the WaitForConditionCommand will
         * be cancelled when the button is let go, but the scoring command (if already scheduled) will be
         * allowed to finish.
         */
        super(
            () -> { // <-- Notice that we're starting a lambda here. This will cast to a Supplier<Command>.
                if(elevator.isTargeting(Elevator.Positions.AMP)){
                    // This is the first instance of the logic described above
                    return new WaitForConditionCommand(
                        () -> elevator.isAmp(),
                        new AmpScoreCommand(shooter, elevator, intake, leds)
                    );
                }
                else{ // If the elevator isn't at (or going to) the amp position, run logic for shooting instead
                    if(forceShoot.getAsBoolean()){ // If we're force-shooting

                        // Ignore whatever else might have been happening and just shoot
                        return new OverrideEverythingCommand(
                            new ShootCommand(shooter, elevator, intake, leds)
                        );
                    }
                    else{
                        // This WaitForConditionCommand has similar logic to the one above
                        return new WaitForConditionCommand(
                            () -> shooter.readyToShoot() && elevator.isAtTargetPosition(),
                            new ShootCommand(shooter, elevator, intake, leds)
                            .andThen(new StowCommand(shooter, elevator, intake))
                        );
                    }
                }
            },
            Set.of(shooter, elevator, intake, leds) // Requirements for this command
        );
    }
}
