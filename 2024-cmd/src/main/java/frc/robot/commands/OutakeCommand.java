package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OutakeCommand extends ProxyCommand {
    public OutakeCommand(Shooter shooter, Intake intake){
        super(
            shooter.outakeCommand()
            .alongWith(intake.outakeCommand())
        );
    }
}
