package frc.robot.commands;
import frc.robot.BunnyDropper.States;
import frc.robot.Constants;
import frc.robot.BunnyDropper;

public class BunnyDropperCommand extends Command {
    BunnyDropper bunnyDropper;
    public BunnyDropperCommand(BunnyDropper bDropper) {
        bunnyDropper = bDropper;
    }

    public void initialize() {

    }

    public boolean execute() {
        return bunnyDropper.dropBunny(States.DROP);

    }


    public void shutdown() {
    }

}