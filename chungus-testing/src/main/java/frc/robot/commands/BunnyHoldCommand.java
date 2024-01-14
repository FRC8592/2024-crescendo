package frc.robot.commands;
import frc.robot.BunnyDropper.States;
import frc.robot.BunnyDropper;

public class BunnyHoldCommand extends Command {
    BunnyDropper bunnyDropper;
    public BunnyHoldCommand(BunnyDropper bDropper) {
        bunnyDropper = bDropper;
    }

    public void initialize() {

    }

    public boolean execute() {
        return bunnyDropper.dropBunny(States.HOLD);

    }


    public void shutdown() {
    }

}