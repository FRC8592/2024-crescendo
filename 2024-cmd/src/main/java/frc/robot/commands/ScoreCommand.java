package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.*;

public class ScoreCommand extends ProxyCommand {
    public ScoreCommand(Shooter shooter, Elevator elevator, Intake intake){
        super(
            new ConditionalCommand(
                shooter.fireCommand(), //If the condition (third argument) is true, do this
                shooter.ampScoreCommand().onlyIf(() -> elevator.isAmp()), //Otherwise, do this
                () -> shooter.readyToShoot() //The condition that picks between the first and second arguments
            )
        );
    }
}
