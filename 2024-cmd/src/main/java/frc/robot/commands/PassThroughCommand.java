package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class PassThroughCommand extends WrapperCommand {
    /**
     * Command to stow, then run pass-through indefinitely
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
            new StowCommand(shooter, elevator, intake)
            .andThen(
                intake.commands.intakeCommand()
                .alongWith(shooter.commands.passThroughCommand())
                .alongWith(leds.commands.blinkCommand(LEDS.ORANGE, 4))
            )
        );
    }
}
