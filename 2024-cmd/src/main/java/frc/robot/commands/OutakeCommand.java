package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class OutakeCommand extends WrapperCommand {
    /**
     * Command to outake. Note that this command is independent of what the
     * elevator might be doing.
     *
     * @param shooter
     * @param intake
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public OutakeCommand(Shooter shooter, Intake intake){
        super(
            shooter.commands.outakeCommand()
            .alongWith(intake.commands.outakeCommand())
        );
    }
}
