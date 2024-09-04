package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.singlemotor.SingleMotor;

public class NewtonCommand extends WrapperCommand {
    protected SingleMotor singleMotor;

    public NewtonCommand(Command command){
        super(command);
    }
}
