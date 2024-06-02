package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class OutakeCommand extends WrapperCommand {
    /**
     * Command to outake. Note that this command is independent of what the elevator might be doing
     *
     * @param shooter
     * @param intake
     *
     * @apiNote This command never ends on its own; it must be interrupted to end
     */
    public OutakeCommand(Shooter shooter, Intake intake){
        super(
            shooter.outakeCommand()
            .alongWith(intake.outakeCommand())
        );
    }
}
