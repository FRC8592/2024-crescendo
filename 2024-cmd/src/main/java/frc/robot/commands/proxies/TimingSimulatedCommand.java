package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Robot;

public class TimingSimulatedCommand extends WrapperCommand{
    /**
     * If running in simulation, replaces the passed-in command with a
     * WaitCommand that waits for a length of your choosing.
     *
     * @param command the command to run on a real robot
     * @param time the time to wait (in seconds) if running
     * in simulation
     */
    public TimingSimulatedCommand(Command command, double time){
        super(
            Robot.isReal() ? command : new WaitCommand(time)
        );
    }

    /**
     * If running in simulation, replaces the passed-in command with
     * {@code Commands.none()}
     *
     * @param command the command to run if on a real robot
     */
    public TimingSimulatedCommand(Command command){
        super(
            Robot.isReal() ? command : Commands.none()
        );
    }
}