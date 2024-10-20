package frc.robot.commands.proxies;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.SHARED;
import frc.robot.subsystems.SubsystemCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public abstract class NewtonCommand extends WrapperCommand {
    protected static Swerve swerve = Swerve.getInstance();
    protected static Intake intake = Intake.getInstance();
    protected static Elevator elevator = Elevator.getInstance();
    protected static Shooter shooter = Shooter.getInstance();
    protected static LEDs leds = LEDs.getInstance();

    public NewtonCommand(Command command){
        super(command);
        setName(getClass().getSimpleName());
    }

    public static Command stopSubsystems(SubsystemCommands... toStop){
        Command result = Commands.none();
        for(SubsystemCommands s:toStop){
            result = result.alongWith(s.stopCommand());
        }
        return result;
    }

    public void initialize(){
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Commands/"+getName()+"/IsRunning", true);
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Commands/"+getName()+"/WasInterruptedLastEnd", false);
        super.initialize();
    }

    public void end(boolean interrupted){
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Commands/"+getName()+"/IsRunning", false);
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Commands/"+getName()+"/WasInterruptedLastEnd", interrupted);
        super.end(interrupted);
    }
}
