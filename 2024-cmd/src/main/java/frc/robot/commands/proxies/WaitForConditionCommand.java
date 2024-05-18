package frc.robot.commands.proxies;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class WaitForConditionCommand extends Command{
    private BooleanSupplier condition;
    private Command command;
    private boolean commandIsScheduled;
    /**
     * Proxy command that waits until a condition is {@code true}, then schedules a command.
     * If the condition is still {@code true} when the command finishes, does not reschedule
     * the command. Note that the WaitForConditionCommand does NOT require the subsystems of
     * the passed-in command.
     *
     * @param condition {@code BooleanSupplier}: a lambda that returns whether or not the
     * command should be scheduled.
     * @param command {@code Command}: the command to schedule when {@code condition} returns
     * {@code true}.
     */
    public WaitForConditionCommand(BooleanSupplier condition, Command command){
        this.condition = condition;
        this.command = command;
    }
    public void initialize(){
        this.commandIsScheduled = false;
    }
    public void execute(){
        if(condition.getAsBoolean() && !this.commandIsScheduled){
            CommandScheduler.getInstance().schedule(command);
            this.commandIsScheduled = true;
        }
    }
    public void end(){}
    public boolean isFinished(){
        return this.commandIsScheduled;
    }
}
