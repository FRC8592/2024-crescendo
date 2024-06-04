package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends WrapperCommand {
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
            shooter.commands.fireCommand()
            .alongWith(leds.commands.singleColorCommand(LEDS.OFF))
        );
        addRequirements(elevator, intake);
    }
}
