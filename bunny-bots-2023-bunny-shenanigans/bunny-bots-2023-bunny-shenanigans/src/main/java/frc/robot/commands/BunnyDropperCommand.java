package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.BunnyDropper;

public class BunnyDropperCommand extends Command {
    BunnyDropper bunnyDropper;
    public BunnyDropperCommand() {
        bunnyDropper = new BunnyDropper();
    }

    public void initialize() {

    }

    public boolean execute() {
        return bunnyDropper.dropBunny();
    }
    

    public void shutdown() {
    }

}
