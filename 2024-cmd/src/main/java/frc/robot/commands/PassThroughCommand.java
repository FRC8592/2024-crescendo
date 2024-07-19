package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class PassThroughCommand extends WrapperCommand {
    /**
     * Command to run pass-through indefinitely. Note that the robot will stow before
     * starting to pass-through.
     *
     * @param shooter
     * @param elevator
     * @param intake
     * @param leds
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public PassThroughCommand(Shooter shooter, Elevator elevator, Intake intake, LEDs leds){
        super(
            // Make sure we're stowed before we run anything
            new StowCommand(shooter, elevator, intake)

            // Reusing the intake's intakeCommand may or may not be permanent
            .andThen(
                intake.commands.intakeCommand()
                .alongWith(shooter.commands.passThroughCommand())
                .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 4))
            )
        );
    }
}
